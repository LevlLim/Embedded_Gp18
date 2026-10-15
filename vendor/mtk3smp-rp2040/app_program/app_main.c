/*
 * Copyright (c) 2026 Muhamed Fauzi Bin Abbas
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 *	app_main.c
 *	Demo application for the micro T-Kernel 3.0 RP2040 SMP port.
 *
 *	Four tasks and one pass of every IPC primitive the kernel offers, wired
 *	into a single producer/consumer pipeline so the objects are used the
 *	way they are meant to be used rather than poked in isolation:
 *
 *	    producer  --[fixed memory pool]--> record
 *	              --[mutex]-------------> shared sequence counter
 *	              --[message buffer]----> consumer
 *	    consumer  --[semaphore]---------> credit returned to producer
 *	              --[event flag]--------> monitor woken
 *	    monitor   prints a status line
 *	    blink     LED liveness, independent of all of the above
 *
 *	Under SMP the producer is pinned to processor 1 and the consumer to
 *	processor 2, so every message crosses a core boundary and the printed
 *	processor numbers differ.  Built with SMP=0 the same source runs
 *	unchanged on one core.
 *
 *	Deliberately small.  Read it top to bottom, change one thing, reflash.
 */

#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>

#if TK_SUPPORT_SMP
#include <tk/smp.h>
/* Which processor is the caller on?  Numbered from 1, so processor 1 is
   RP2040 core 0 and processor 2 is core 1. */
#define THIS_PRC()	tk_get_prc()
#else
/* Single-core build: there is only ever processor 1.  tk_get_prc() is
   declared by the kernel headers but not linked in this configuration, so
   the demo asks through this macro and the rest of the source is identical
   for both builds. */
#define THIS_PRC()	1
#endif

#include "usb_console_compat.h"
#include "demo_tasks.h"

#if TM_WIFI_CYW43
/* Plain C types on purpose - see the note in cyw43_utk.h. */
#include "cyw43_utk.h"
#endif

/* ------------------------------------------------------------------ *
 *  Tunables
 * ------------------------------------------------------------------ */

#define N_RECORDS	4		/* blocks in the fixed memory pool  */
#define CREDITS		2		/* messages in flight at once       */
#define PRODUCE_MS	500		/* one record every half second     */
#define REPORT_MS	2000		/* monitor prints this often        */

#define FLG_PRODUCED	(1U << 0)
#define FLG_CONSUMED	(1U << 1)

/* Every task uses 4096 bytes.  The SMP kernel path is deeper than the
   single-core one -- each critical section runs the global ready-queue
   assignment on the calling task's stack -- so 1-2 KiB is no longer enough
   for a task that makes blocking kernel calls. */
#define STACK_SZ	4096

/* ------------------------------------------------------------------ *
 *  One record travelling through the pipeline
 * ------------------------------------------------------------------ */

typedef struct {
	UW	seq;			/* sequence number, mutex-guarded    */
	INT	made_on;		/* processor that produced it        */
	UW	payload;		/* something to check on arrival     */
} RECORD;

/* Kernel object ids, filled in by usermain(). */
LOCAL ID	mpfid;			/* fixed-size memory pool            */
LOCAL ID	mtxid;			/* guards next_seq                   */
LOCAL ID	mbfid;			/* producer -> consumer              */
LOCAL ID	semid;			/* consumer -> producer (credits)    */
LOCAL ID	flgid;			/* both -> monitor                   */

/* Shared state.  next_seq is guarded by the mutex; the counters below are
   written by one task each and only read by the monitor, so they need no
   lock of their own. */
LOCAL UW	next_seq;
LOCAL UW	produced, consumed, dropped;
LOCAL INT	last_made_on, last_seen_on;

/* Backing store for the pool and the message buffer.  Static, because a
   kernel object must not own memory that can go out of scope. */
LOCAL UW	mpf_buf[(N_RECORDS * sizeof(RECORD) + sizeof(UW) - 1) / sizeof(UW)
			+ N_RECORDS];
LOCAL UW	mbf_buf[(CREDITS * sizeof(RECORD)) / sizeof(UW) + CREDITS * 4];

/* ------------------------------------------------------------------ *
 *  Producer -- pinned to processor 1 under SMP
 * ------------------------------------------------------------------ */

LOCAL void producer_task(INT stacd, void *exinf)
{
	RECORD	*rec;
	RECORD	msg;
	ER	er;

	(void)stacd; (void)exinf;

	while(1) {
		/* Wait for a credit: the consumer returns one per message, so
		   the producer can never outrun it by more than CREDITS. */
		er = tk_wai_sem(semid, 1, 1000);
		if(er < E_OK) { dropped++; continue; }

		/* A block from the fixed-size pool, rather than a local, to
		   show tk_get_mpf/tk_rel_mpf round-tripping. */
		er = tk_get_mpf(mpfid, (void **)&rec, 100);
		if(er < E_OK) { dropped++; tk_sig_sem(semid, 1); continue; }

		/* The sequence counter is the one piece of state two tasks
		   could race on, so it is the one thing under a mutex. */
		tk_loc_mtx(mtxid, TMO_FEVR);
		rec->seq = ++next_seq;
		tk_unl_mtx(mtxid);

		rec->made_on = THIS_PRC();
		rec->payload = rec->seq * 7U;

		msg = *rec;			/* copy out before releasing */
		tk_rel_mpf(mpfid, rec);

		er = tk_snd_mbf(mbfid, &msg, (INT)sizeof msg, 1000);
		if(er < E_OK) { dropped++; tk_sig_sem(semid, 1); continue; }

		produced++;
		last_made_on = msg.made_on;
		tk_set_flg(flgid, FLG_PRODUCED);

		tk_dly_tsk(PRODUCE_MS);
	}
}

/* ------------------------------------------------------------------ *
 *  Consumer -- pinned to processor 2 under SMP
 * ------------------------------------------------------------------ */

LOCAL void consumer_task(INT stacd, void *exinf)
{
	RECORD	msg;
	INT	sz;

	(void)stacd; (void)exinf;

	while(1) {
		sz = tk_rcv_mbf(mbfid, &msg, TMO_FEVR);
		if(sz != (INT)sizeof msg) { dropped++; continue; }

		/* Cheap integrity check: the payload is a known function of
		   the sequence number, so a torn or reordered message shows
		   up immediately. */
		if(msg.payload != msg.seq * 7U) { dropped++; continue; }

		consumed++;
		last_seen_on = THIS_PRC();
		tk_set_flg(flgid, FLG_CONSUMED);

		/* Return the credit. */
		tk_sig_sem(semid, 1);
	}
}

/* ------------------------------------------------------------------ *
 *  Monitor -- woken by the event flag, prints a line
 * ------------------------------------------------------------------ */

LOCAL void monitor_task(INT stacd, void *exinf)
{
	UINT	ptn;
	ER	er;
	INT	i;

	(void)stacd; (void)exinf;

	/* On a USB-CDC build the console is a 4 KB ring drained by the host.
	   Anything printed before the host enumerates goes into a ring nobody
	   is reading and is lost, so wait for the link first.  Bounded at ~20 s
	   so a headless board still runs; on a UART build tm_usb_state() is a
	   constant and this falls straight through. */
	for(i = 0; i < 200 && tm_usb_state() < 2; i++) {
		tk_dly_tsk(100);
	}
	tk_dly_tsk(500);

	tm_printf((UB *)"\n=== uT-Kernel 3.0 / RP2040 demo ===\n");
#if TK_SUPPORT_SMP
	tm_printf((UB *)"SMP build: %d processors\n", TK_MAX_CORE);
#else
	tm_printf((UB *)"single-core build\n");
#endif
	tm_printf((UB *)"producer -> [mbf] -> consumer, %d credits, %d ms period\n\n",
		  CREDITS, PRODUCE_MS);

	while(1) {
		/* Wait until both ends have moved at least once since the
		   last report.  TWF_ANDW = wait for every bit; the flag is
		   cleared as part of the wait so the next round starts
		   clean. */
		er = tk_wai_flg(flgid, FLG_PRODUCED | FLG_CONSUMED,
				TWF_ANDW | TWF_CLR, &ptn, REPORT_MS);

		if(er == E_TMOUT) {
			tm_printf((UB *)"[monitor] stalled: produced=%u consumed=%u\n",
				  produced, consumed);
			continue;
		}

		tm_printf((UB *)"[monitor] seq=%u produced=%u consumed=%u dropped=%u"
			       "  made_on=prc%d seen_on=prc%d\n",
			  next_seq, produced, consumed, dropped,
			  last_made_on, last_seen_on);

		/* One line every REPORT_MS cannot overrun a 4 KB ring, but say
		   so out loud if the console ever does drop bytes. */
		if(tm_usb_dropped_bytes() != 0) {
			tm_printf((UB *)"[monitor] console dropped %u bytes\n",
				  tm_usb_dropped_bytes());
		}

		tk_dly_tsk(REPORT_MS);
	}
}

#if TM_WIFI_CYW43
/* ------------------------------------------------------------------ *
 *  WiFi status -- only built when WIFI=cyw43
 *
 *  The radio is not driven from here.  A dedicated service task owns the
 *  CYW43439, lwIP and the echo session -- pinned to processor 1 under SMP,
 *  and simply the only processor there is on a single-core build.  This task
 *  only reads the snapshot that service publishes and prints what changed.
 *  Reading is safe from any task: cyw43_utk_get_status() takes a seqlock
 *  copy, which is why the application goes through it rather than calling
 *  lwip_utk_udp_get_status() directly.
 * ------------------------------------------------------------------ */

#define WIFI_IDLE_MS	3000	/* nothing happening: report rarely      */
#define WIFI_BUSY_MS	 200	/* session running: drain the echo ring  */

/* Compose, then emit.  tm_printf is atomic per call, but a line built from
   several calls can still be spliced by a higher-priority task printing
   between them -- so every line below is formatted into a local buffer and
   emitted with exactly one call. */
LOCAL void fmt_ipv4(UB *dst, const UB *a)
{
	tm_sprintf(dst, (UB *)"%u.%u.%u.%u", a[0], a[1], a[2], a[3]);
}

LOCAL void wifi_status_task(INT stacd, void *exinf)
{
	T_CYW43_UTK_STATUS	st;
	UW			last_link = 0xffffffffU;
	UW			cursor = 0;	/* echoes printed so far */
	UB			ip[16];
	UW			last_dhcp = 0xffffffffU;
	UW			stalled_polls = 0;
	UW			shown_session = 0xffffffffU;
	UW			summarised = 0xffffffffU;
	INT			i;

	(void)stacd; (void)exinf;

	/* Same enumeration wait the monitor uses.  Without it this task races
	   ahead of the banner and its first lines land before the demo has
	   introduced itself. */
	for(i = 0; i < 200 && tm_usb_state() < 2; i++) {
		tk_dly_tsk(100);
	}
	tk_dly_tsk(700);		/* let the monitor print its banner first */

	while(1) {
		cyw43_utk_get_status(&st);

		/* Link transitions, printed once each rather than every tick. */
		if(st.link_up != last_link) {
			last_link = st.link_up;
			if(st.link_up) {
				tm_printf((UB *)"[wifi] link UP  rssi=%d  mac=%02x:%02x:%02x:%02x:%02x:%02x\n",
					  st.link_rssi, st.mac[0], st.mac[1],
					  st.mac[2], st.mac[3], st.mac[4], st.mac[5]);
				/* The address is reported by its own transition
				   below -- DHCP usually finishes after this. */
			} else {
				tm_printf((UB *)"[wifi] link DOWN\n");
			}
		}

		/* DHCP completes some time after the link comes up, so report it
		   on its own transition.  Without this the address is only ever
		   printed in the rare case it lands in the same poll as link-up,
		   and a run that never gets an address looks like silence. */
		if(st.dhcp_complete != last_dhcp) {
			last_dhcp = st.dhcp_complete;
			if(st.dhcp_complete) {
				UB gw[16], dns[16];
				fmt_ipv4(ip, st.dhcp_address);
				fmt_ipv4(gw, st.dhcp_gateway);
				fmt_ipv4(dns, st.dhcp_dns);
				tm_printf((UB *)"[wifi] ip=%s  gw=%s  dns=%s\n",
					  ip, gw, dns);
			}
		}

		if(!st.udp_enabled) {
			tk_dly_tsk(WIFI_IDLE_MS);
			continue;
		}

		/* Nothing sent yet?  Say what is still missing rather than going
		   quiet.  The session cannot start until the interface is up and
		   has an address. */
		if(st.udp_packets_sent == 0 && st.udp_write_seq == 0) {
			if(++stalled_polls == 25U) {	/* ~5 s at the busy rate */
				tm_printf((UB *)"[udp] waiting: link=%u netif_up=%u"
					       " link_up=%u dhcp=%u addr=%u.%u.%u.%u\n",
					  st.link_up, st.netif_up, st.netif_link_up,
					  st.dhcp_complete, st.dhcp_address[0],
					  st.dhcp_address[1], st.dhcp_address[2],
					  st.dhcp_address[3]);
				stalled_polls = 0;
			}
		} else {
			stalled_polls = 0;
		}

		/* Announce the session once, when the first packet goes out.
		   Before that the target is still 0.0.0.0:0 and printing it is
		   noise, not information. */
		if(st.udp_packets_sent != 0 && st.udp_session_count != shown_session) {
			shown_session = st.udp_session_count;
			fmt_ipv4(ip, st.udp_target);
			tm_printf((UB *)"[udp] session %u: echoing to %s:%u,"
				       " %u packets of %u bytes\n",
				  st.udp_session_count + 1U, ip, st.udp_port,
				  st.udp_expected_packets, st.udp_payload_size);
		}

		/* Drain the ring: print every echo that arrived since last look.
		   If the reader ever falls more than LWIP_UTK_UDP_RING behind,
		   skip forward and say so rather than printing stale slots. */
		if(st.udp_write_seq > cursor) {
			if(st.udp_write_seq - cursor > CYW43_UDP_RING) {
				tm_printf((UB *)"[udp] (%u echoes not shown, reader behind)\n",
					  st.udp_write_seq - cursor - CYW43_UDP_RING);
				cursor = st.udp_write_seq - CYW43_UDP_RING;
			}
			while(cursor < st.udp_write_seq) {
				UW	slot = cursor % CYW43_UDP_RING;
				UB	line[96];
				INT	n;

				n = tm_sprintf(line, (UB *)"[udp] echo #%u, %u bytes:",
					       st.udp_ring_seq[slot],
					       st.udp_ring_len[slot]);
				for(i = 0; i < 12 && i < (INT)st.udp_ring_len[slot]; i++) {
					n += tm_sprintf(&line[n], (UB *)" %02x",
							st.udp_ring_payload[slot][i]);
				}
				if(st.udp_ring_len[slot] > 12) {
					n += tm_sprintf(&line[n], (UB *)" ...");
				}
				tm_printf((UB *)"%s\n", line);
				cursor++;
			}
		}

		if(st.udp_complete && st.udp_session_count != summarised) {
			summarised = st.udp_session_count;
			fmt_ipv4(ip, st.udp_target);
			tm_printf((UB *)"[udp] %s:%u  sent=%u recv=%u matched=%u/%u"
				       "  corrupt=%u retries=%u errs=%u  %u ms\n",
				  ip, st.udp_port, st.udp_packets_sent,
				  st.udp_packets_received, st.udp_packets_matched,
				  st.udp_expected_packets, st.udp_corrupt,
				  st.udp_retries, st.udp_send_errors,
				  st.udp_elapsed_ms);
			tm_printf((UB *)"[udp] session %u complete: result=%d%s\n",
				  st.udp_session_count + 1U, st.udp_result,
				  (st.udp_packets_matched == st.udp_expected_packets
				   && st.udp_corrupt == 0)
				  ? "  ALL PACKETS ECHOED" : "");
		}

		/* Poll fast whenever a session could be running, not only once
		   one has been observed running.  A session lasts about 1.4 s
		   and repeats every 5 s; at the idle rate this task would sleep
		   straight through one and find the ring already wrapped.  Once
		   the link is up and UDP is enabled, stay at the fast rate. */
		tk_dly_tsk((st.udp_enabled && st.link_up)
			   ? WIFI_BUSY_MS : WIFI_IDLE_MS);
	}
}

LOCAL T_CTSK ctsk_wifi = {
	.itskpri	= 8,
	.stksz		= STACK_SZ,
	.task		= wifi_status_task,
	.tskatr		= TA_HLNG | TA_RNG3,
};
#endif	/* TM_WIFI_CYW43 */

/* ------------------------------------------------------------------ *
 *  Task and object definitions
 * ------------------------------------------------------------------ */

LOCAL T_CTSK ctsk_producer = {
	.itskpri	= 5,
	.stksz		= STACK_SZ,
	.task		= producer_task,
#if TK_SUPPORT_SMP
	.tskatr		= TA_HLNG | TA_RNG3 | TA_ASSPRC,
	.assprc		= TP_PRC1,
#else
	.tskatr		= TA_HLNG | TA_RNG3,
#endif
};

LOCAL T_CTSK ctsk_consumer = {
	.itskpri	= 6,
	.stksz		= STACK_SZ,
	.task		= consumer_task,
#if TK_SUPPORT_SMP
	.tskatr		= TA_HLNG | TA_RNG3 | TA_ASSPRC,
	.assprc		= TP_PRC2,
#else
	.tskatr		= TA_HLNG | TA_RNG3,
#endif
};

LOCAL T_CTSK ctsk_monitor = {
	.itskpri	= 7,
	.stksz		= STACK_SZ,
	.task		= monitor_task,
	.tskatr		= TA_HLNG | TA_RNG3,
};

LOCAL T_CTSK ctsk_blink = {
	.itskpri	= 10,
	.stksz		= STACK_SZ,
	.task		= blink_task,
	.tskatr		= TA_HLNG | TA_RNG3,
};

LOCAL T_CMPF cmpf = {
	.mpfatr		= TA_TFIFO | TA_RNG3,
	.mpfcnt		= N_RECORDS,
	.blfsz		= sizeof(RECORD),
	.bufptr		= mpf_buf,
};

LOCAL T_CMTX cmtx = {
	.mtxatr		= TA_TFIFO | TA_INHERIT,
};

LOCAL T_CMBF cmbf = {
	.mbfatr		= TA_TFIFO,
	.bufsz		= sizeof mbf_buf,
	.maxmsz		= sizeof(RECORD),
	.bufptr		= mbf_buf,
};

LOCAL T_CSEM csem = {
	.sematr		= TA_TFIFO,
	.isemcnt	= CREDITS,
	.maxsem		= CREDITS,
};

LOCAL T_CFLG cflg = {
	.flgatr		= TA_TFIFO | TA_WMUL,
	.iflgptn	= 0,
};

/* ------------------------------------------------------------------ *
 *  Entry point
 * ------------------------------------------------------------------ */

/* Create an object, or say which one failed and stop.  A silent object
   failure here would surface much later as a mysterious hang. */
LOCAL BOOL made(const char *what, ID id)
{
	if(id <= E_OK) {
		tm_printf((UB *)"[init] tk_cre_%s failed, er=%d\n", what, id);
		return FALSE;
	}
	return TRUE;
}

EXPORT INT usermain(void)
{
	ID	tid;

	mpfid = tk_cre_mpf(&cmpf); if(!made("mpf", mpfid)) return 1;
	mtxid = tk_cre_mtx(&cmtx); if(!made("mtx", mtxid)) return 1;
	mbfid = tk_cre_mbf(&cmbf); if(!made("mbf", mbfid)) return 1;
	semid = tk_cre_sem(&csem); if(!made("sem", semid)) return 1;
	flgid = tk_cre_flg(&cflg); if(!made("flg", flgid)) return 1;

	tid = tk_cre_tsk(&ctsk_blink);    if(made("tsk(blink)",    tid)) tk_sta_tsk(tid, 0);
	tid = tk_cre_tsk(&ctsk_monitor);  if(made("tsk(monitor)",  tid)) tk_sta_tsk(tid, 0);
	tid = tk_cre_tsk(&ctsk_consumer); if(made("tsk(consumer)", tid)) tk_sta_tsk(tid, 0);
	tid = tk_cre_tsk(&ctsk_producer); if(made("tsk(producer)", tid)) tk_sta_tsk(tid, 0);
#if TM_WIFI_CYW43
	tid = tk_cre_tsk(&ctsk_wifi);     if(made("tsk(wifi)",     tid)) tk_sta_tsk(tid, 0);
#endif

	/* The initial task has nothing left to do.  It must not return: on
	   return the kernel shuts the system down. */
	tk_slp_tsk(TMO_FEVR);
	return 0;
}
