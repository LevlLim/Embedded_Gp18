/*
 * app.h
 *
 * Group AAI18 -- definitions shared by the whole team.
 * Changed only by team agreement.
 *
 * Anything that belongs to ONE subsystem goes in that subsystem's own
 * header instead (app_program/subsystemN/subsystemN.h).
 *
 * Subsystem map (Design Review V2 section 12.3):
 *   subsystem1/  Buddy 1  Telemetry / Comms      WiFi, MQTT, remote cmds
 *   subsystem2/  Buddy 2  Motion Control         motors, encoders, PID
 *   subsystem3/  Buddy 3  Line + Barcode         ADC sampler, IR, decode
 *   subsystem4/  Buddy 4  IMU / Terrain          hump, tilt, collision
 *   subsystem5/  Buddy 5  Obstacle + Controller  ultrasonic, mission state
 */

#ifndef APP_H
#define APP_H

/*
 * The C library headers come FIRST, and the kernel is told not to define
 * its own size_t.  Without PROHIBIT_DEF_SIZE_T the kernel's
 * "typedef SZ size_t" (include/tk/syslib.h) collides with <stddef.h>, and
 * without <stdint.h> here uint32_t below is an unknown type.  Both errors
 * only appear against the real template headers, not the host stubs.
 */
#define PROHIBIT_DEF_SIZE_T
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include <tk/tkernel.h>     /* micro T-Kernel API (tk_xxx_yyy)        */
#include <tm/tmonitor.h>    /* T-Monitor API: tm_printf()             */
#include <bsp/libbsp.h>     /* Board helpers: gpio_set_pin/val, pwm_  */

/* ---------------------------------------------------------------------
 *  KERNEL TICK.  Every periodic task below runs from a cyclic handler, so
 *  its period can be no finer than the kernel tick.  The template ships
 *  with CNF_TIMER_PERIOD 10 (config/config.h); at that setting the "1 ms"
 *  barcode scan and the "5 ms" line task cannot exist.  Fail the build
 *  rather than run with silently wrong timing.
 * --------------------------------------------------------------------- */
#if defined(TIMER_PERIOD) && (TIMER_PERIOD != 1)
#error "Set CNF_TIMER_PERIOD to 1 in config/config.h (see template_patches)"
#endif

/* ---------------------------------------------------------------------
 *  PIN MAP -- Design Review V2 section 2.  One owner per pin.
 *  Never use GP23/24/25/29 (Pico W radio).
 *
 *  ULTRASONIC ON GP0/GP1: the HC-SR04P is wired to GP0 (TRIG) and GP1
 *  (ECHO), the pins the template otherwise uses for its UART0 console.
 *  The template leaves them free only in the USB-console build with
 *  CRASH_REPORT=0 (it then defines BOARD_UART0_FREE=1).  Any build that
 *  puts the console or the crash mirror on UART0 would fight the sensor
 *  for those pins, so it is refused below.
 * --------------------------------------------------------------------- */
#if defined(TM_CONSOLE_UART) && (TM_CONSOLE_UART != 0)
#error "Ultrasonic is on GP0/GP1: use the USB console, CRASH_REPORT=0"
#endif

#define PIN_ENC_L_A         (2u)    /* S2  left encoder phase A  (IRQ)   */
#define PIN_ENC_L_B         (3u)    /* S2  left encoder phase B          */
#define PIN_ENC_R_A         (4u)    /* S2  right encoder phase A (IRQ)   */
#define PIN_ENC_R_B         (5u)    /* S2  right encoder phase B         */
#define PIN_US_TRIG         (0u)    /* S5  HC-SR04P trigger              */
#define PIN_US_ECHO         (1u)    /* S5  HC-SR04P echo         (IRQ)   */
#define PIN_MOTOR_L_A       (8u)    /* S2  PWM slice 4 channel A         */
#define PIN_MOTOR_L_B       (9u)    /* S2  PWM slice 4 channel B         */
#define PIN_MOTOR_R_A       (10u)   /* S2  PWM slice 5 channel A         */
#define PIN_MOTOR_R_B       (11u)   /* S2  PWM slice 5 channel B         */
#define PIN_SERVO_SCAN      (12u)   /* S5  PWM slice 6, 50 Hz            */
#define PIN_IMU_SDA         (16u)   /* S4  I2C0 SDA                      */
#define PIN_IMU_SCL         (17u)   /* S4  I2C0 SCL                      */
#define PIN_STATUS_LED      (18u)   /* S5  WS2812 status LED             */
#define PIN_BTN_START       (20u)   /* S5  onboard start button          */
#define PIN_BTN_STOP        (21u)   /* S5  onboard stop button           */
#define PIN_BUZZER          (22u)   /* S5  PWM slice 3                   */
#define PIN_IR_LINE_L       (26u)   /* S3  ADC0  -- owned by adc_sampler */
#define PIN_IR_LINE_C       (27u)   /* S3  ADC1  -- owned by adc_sampler */
#define PIN_IR_BARCODE      (28u)   /* S3  ADC2  -- owned by adc_sampler */

/* ---------------------------------------------------------------------
 *  TASK PRIORITIES -- Design Review V2 section 3.
 *  1 is the highest.  The kernel's USB console (3) and WiFi (4) tasks sit
 *  above every application task, so application bands start at 5.
 *
 *  NOTE: the WiFi/lwIP service task is pinned to core 0 by the template
 *  (cyw43_utk_kernel.c), the same core as the driving tasks, and at a
 *  HIGHER priority than Motion Control.  NFR6 therefore rests on that task
 *  being short, not on core separation.  Measure it (test T6).
 * --------------------------------------------------------------------- */
#define PRI_MOTION_CONTROL  (5)     /* Highest: 10 ms PID + e-stop brake */
#define PRI_LINE_SENSOR     (7)     /* High:    5 ms                     */
#define PRI_BARCODE         (8)     /* High:    1 ms scan                */
#define PRI_IMU_TERRAIN     (9)     /* High:    10 ms, 4 ms WCET         */
#define PRI_VEHICLE_CTRL    (12)    /* Medium:  event driven             */
#define PRI_OBSTACLE_SCAN   (13)    /* Medium:  60 ms (sensor rearm)     */
#define PRI_TELEMETRY       (20)    /* Low:     never delays driving     */

#define TASK_STACK_SIZE     (2048)  /* bytes; tm_printf needs ~1 KB      */

/* ---------------------------------------------------------------------
 *  TIMING -- periods in milliseconds.  Each periodic task is woken by its
 *  own cyclic handler (app_start_periodic_task), so the period does not
 *  drift with the task's own execution time.
 * --------------------------------------------------------------------- */
#define PERIOD_MOTION_MS    (10u)
#define PERIOD_LINE_MS      (5u)
#define PERIOD_BARCODE_MS   (1u)
#define PERIOD_IMU_MS       (10u)
#define PERIOD_OBSTACLE_MS  (60u)   /* HC-SR04P minimum rearm interval   */
#define PERIOD_TELEMETRY_MS (200u)  /* 5 Hz, satisfies FR9               */

/* ---------------------------------------------------------------------
 *  PROVISIONAL CONSTANTS.
 *  Every value here is an engineering estimate, not a measurement.  Each
 *  must be replaced from bench data before Week 10 -- see Design Review V2
 *  section 14, ordered actions 3 and 5.  They are gathered in one place so
 *  that it is obvious what has not yet been measured.
 * --------------------------------------------------------------------- */
#define PROV_D_DETECT_CM    (30u)   /* FR6  obstacle detection distance  */
#define PROV_D_STOP_CM      (20u)   /* NFR2 hard stop distance           */
#define PROV_CRUISE_PCT     (40u)   /* nominal cruising speed, % duty    */
#define PROV_DEGRADE_PCT    (30u)   /* speed cap when a DEGRADE fault is */
                                    /* active (e.g. IMU_FAULT)           */

/* ---------------------------------------------------------------------
 *  MESSAGE BUFFER SIZING.
 *  A micro T-Kernel message buffer stores every message behind a 4-byte
 *  header (HEADER is an INT) and rounds the body up to a multiple of 4
 *  (kernel/tkernel/messagebuf.h).  A buffer sized as depth x message
 *  length therefore holds far fewer messages than intended -- for a
 *  1-byte message it holds none at all.  Always size with this macro.
 * --------------------------------------------------------------------- */
#define APP_MBF_SLOT_BYTES(msg_len)  (4u + ((((msg_len) + 3u) / 4u) * 4u))
#define APP_MBF_BUFSZ(depth, msg_len) \
    ((depth) * APP_MBF_SLOT_BYTES(msg_len))

/* ---------------------------------------------------------------------
 *  Create and start a task.  prc = TP_PRC1 (core 0), TP_PRC2 (core 1), or
 *  0 for "any core".  Defined in app_main.c.
 * --------------------------------------------------------------------- */
IMPORT ID app_start_task(FP entry, PRI pri, UW prc, const char *p_name);

/*
 * Create and start a PERIODIC task: the task is created as above, then a
 * cyclic handler wakes it every period_ms.  The task body loops on
 * app_wait_period().  Returns the task ID or a negative error.
 */
IMPORT ID app_start_periodic_task(FP entry, PRI pri, UW prc,
                                  const char *p_name, uint32_t period_ms);

/*
 * Block until the next period.  Returns the number of wake-ups that were
 * missed because the previous cycle overran (0 in normal operation).
 * Missed wake-ups are discarded rather than replayed back to back.
 */
IMPORT uint32_t app_wait_period(void);

/* Milliseconds since boot, from the kernel tick.  Used for the staleness
   timestamps that Motion Control checks on line_error. */
IMPORT uint32_t app_now_ms(void);

/* ---------------------------------------------------------------------
 *  Every subsystem provides these.  usermain() calls ALL the _init()
 *  functions first and only then ALL the _start() functions, so every
 *  kernel object exists before any task can use it.
 * --------------------------------------------------------------------- */
IMPORT ER subsystem1_init(void);
IMPORT ER subsystem2_init(void);
IMPORT ER subsystem3_init(void);
IMPORT ER subsystem4_init(void);
IMPORT ER subsystem5_init(void);

IMPORT ER subsystem1_start(void);
IMPORT ER subsystem2_start(void);
IMPORT ER subsystem3_start(void);
IMPORT ER subsystem4_start(void);
IMPORT ER subsystem5_start(void);

/* Which buddy's self-test a TEST build runs: 0 = all, 1..5 = that buddy
   only.  Set by the build (TEST_BUDDY=N / AAI18_TEST_BUDDY). */
#ifndef APP_TEST_BUDDY
#define APP_TEST_BUDDY  (0)
#endif
#if (APP_TEST_BUDDY < 0) || (APP_TEST_BUDDY > 5)
#error "APP_TEST_BUDDY must be 0 (all) or 1..5"
#endif

#if APP_TEST
IMPORT void subsystem1_test(void);
IMPORT void subsystem2_test(void);
IMPORT void subsystem3_test(void);
IMPORT void subsystem4_test(void);
IMPORT void subsystem5_test(void);
#endif

/* ---------------------------------------------------------------------
 *  LOG LINES -- console AND Wi-Fi.
 *  APP_LOG(fmt, ...) prints exactly like tm_printf, and in an MQTT build
 *  also sends the line to topic car18/log, so the car's console can be
 *  read on a laptop with no USB cable:
 *      mosquitto_sub -h <broker> -t car18/log
 *  Used for the event lines ([S3]..[S5], [DBG]); a line longer than
 *  APP_LOG_MAX is cut short on Wi-Fi (the console gets it whole).
 *  A macro (Barr-C deviation D5, as CHECK below) because the template's
 *  printf has no va_list form to forward the arguments to.
 * --------------------------------------------------------------------- */
#define APP_LOG_MAX     (120u)

IMPORT void app_log_line(const UB *p_text);     /* subsystem1 */

#define APP_LOG(...)                                                    \
    do                                                                  \
    {                                                                   \
        UB app_log_buf_[APP_LOG_MAX + 64u];                             \
        app_log_buf_[0] = (UB)0;                                        \
        (void)tm_sprintf(app_log_buf_, __VA_ARGS__);                    \
        app_log_line(app_log_buf_);                                     \
    } while (0)

/* Print a readable message if a kernel call failed (negative error code).
   Kept as a macro (Barr-C 6.3a deviation D5) so the call site's text can
   be passed through unchanged. */
#define CHECK(er, what)                                                 \
    do                                                                  \
    {                                                                   \
        INT check_er_ = (INT)(er);      /* evaluate once */             \
        if (check_er_ < E_OK)                                           \
        {                                                               \
            tm_printf((UB*)"ERROR %s: %d\n", (what), check_er_);        \
        }                                                               \
    } while (0)

#endif /* APP_H */

/* end of file */
