/* Stub micro T-Kernel header: HOST SYNTAX CHECK ONLY.
   Not part of the firmware build. Declares just enough of the API that
   app_program/ uses so gcc -fsyntax-only can parse our sources. */
#ifndef STUB_TKERNEL_H
#define STUB_TKERNEL_H
#include <stdint.h>
#include <stddef.h>
typedef int INT; typedef unsigned int UINT; typedef int ER; typedef int ID;
typedef int PRI; typedef int TMO; typedef unsigned int RELTIM; typedef unsigned int ATR; typedef unsigned int UW; typedef unsigned char UB;
typedef unsigned short UH; typedef int BOOL; typedef void (*FP)(void);
typedef volatile unsigned int _UW; typedef volatile unsigned short _UH;
typedef volatile unsigned char _UB;
#define EXPORT
#define IMPORT extern
#define WEAK_FUNC
#define Inline static inline
#define TRUE 1
#define FALSE 0
#define E_OK 0
#define E_PAR (-17)
#define E_OBJ (-41)
#define E_IO  (-27)
#define E_TMOUT (-50)
#define TMO_POL 0
#define TMO_FEVR (-1)
#define TA_HLNG 0x01
#define TA_RNG3 0x300
#define TA_ASSPRC 0x4000
#define TA_TFIFO 0x00
#define TA_WMUL 0x02
#define TA_INHERIT 0x02
#define TWF_ORW 0x01
#define TWF_CLR 0x10
#define TP_PRC1 1
#define TP_PRC2 2
#define TK_SUPPORT_SMP 1
#define TIMER_PERIOD 1
#define TSK_SELF 0
#define TA_STA 0x02
typedef struct { int hi; UW lo; } SYSTIM;
typedef struct { FP task; PRI itskpri; INT stksz; UW tskatr; UW assprc; } T_CTSK;
typedef struct { void *exinf; ATR cycatr; FP cychdr; RELTIM cyctim; RELTIM cycphs; } T_CCYC;
typedef struct { UW flgatr; UINT iflgptn; } T_CFLG;
typedef struct { UINT flgptn; } T_RFLG;
typedef struct { UW mbfatr; INT bufsz; INT maxmsz; } T_CMBF;
typedef struct { UW mtxatr; PRI ceilpri; } T_CMTX;
typedef struct { UW intatr; FP inthdr; } T_DINT;
typedef struct { UH spver; UH prver; } T_RVER;
ID tk_cre_tsk(T_CTSK*); ER tk_sta_tsk(ID,INT); ER tk_slp_tsk(TMO);
ER tk_dly_tsk(INT); ER tk_get_tim(SYSTIM*); UINT tk_get_prc(void);
ER tk_ref_ver(T_RVER*);
ID tk_cre_cyc(const T_CCYC*); ER tk_wup_tsk(ID); INT tk_can_wup(ID);
ID tk_cre_flg(T_CFLG*); ER tk_set_flg(ID,UINT); ER tk_clr_flg(ID,UINT);
ER tk_wai_flg(ID,UINT,UINT,UINT*,TMO); ER tk_ref_flg(ID,T_RFLG*);
ID tk_cre_mbf(T_CMBF*); ER tk_snd_mbf(ID,void*,INT,TMO); INT tk_rcv_mbf(ID,void*,TMO);
ID tk_cre_mtx(T_CMTX*); ER tk_loc_mtx(ID,TMO); ER tk_unl_mtx(ID);
ER tk_def_int(UINT,T_DINT*); void EnableInt(UINT,INT);
UINT disint(void); void set_primask(UINT);
#define DI(i) ((i)=disint())
#define EI(i) (set_primask(i))
Inline void out_w(UW p, UW d){ *(_UW*)(size_t)p = d; }
Inline UW in_w(UW p){ return *(_UW*)(size_t)p; }
Inline void clr_w(UW a, UW d){ *(_UW*)(size_t)(a+0x3000) = d; }
Inline void set_w(UW a, UW d){ *(_UW*)(size_t)(a+0x2000) = d; }
#endif
