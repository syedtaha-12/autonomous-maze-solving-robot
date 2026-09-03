/*******************************************************************
 * Autonomous Maze-Solving Robot (MC9S12C32)
 *
 * My firmware for a robot that line-follows a maze track, explores
 * unknown branches, backtracks out of dead ends, and retraces the
 * shortest learned path home once it reaches the destination.
 *
 * Design notes:
 *
 *  - DISPATCHER is a plain switch: exactly one state handler runs
 *    per call.
 *
 *  - In FWD_ST, the front bumper (PORTAD0 bit 0x04) signals a dead
 *    end and the rear bumper (bit 0x08) signals that the operator
 *    has reached the forward destination - these are checked
 *    separately so each has its own, distinct behavior.
 *
 *  - UPDT_DISPL looks up the current state's display name from
 *    `tab`, an array of string pointers indexed by state number.
 *
 *  - openADC() and initAD() are both called from _Startup and both
 *    write ATDCTL2/3/4 - initAD() runs second, so its settings are
 *    what actually take effect; openADC()'s are immediately
 *    overwritten. Left as-is (harmless, just redundant).
 *
 *  - DISPLAY_SENSORS/BIN2ASC/TOP_LINE/BOT_LINE build a debug readout
 *    of the raw sensor values but I never call them from MAIN. I'm
 *    keeping them here as real, working functions (not deleted)
 *    since my project spec explicitly recommends a sensor readout
 *    on the LCD - I can call DISPLAY_SENSORS() from main()'s loop
 *    if I want that instead of/alongside UPDT_DISPL().
 *
 *  - del_50us() is a simple busy-wait delay, not a cycle-accurate
 *    timing loop.
 *
 * Navigation manager:
 *
 *  FWD_ST handles reactive line-following and steers onto whichever
 *  branch a junction sensor detects. On top of that, my navigation
 *  manager handles the maze-solving logic my project spec requires:
 *  counting intersections, remembering which branch I took at each
 *  one, backtracking and trying the other branch when a branch
 *  dead-ends, and retracing the learned path home once the rear
 *  bumper signals the forward destination.
 *
 *   - Every time FWD_ST is about to commit to a branch turn
 *     (PARTIAL_LEFT_TRN / PARTIAL_RIGHT_TRN), NAV_ENTER_BRANCH() records
 *     which side I took in MAZE_CHOICE[], indexed by intersection
 *     number (MAZE_COUNT).
 *   - On a dead end (front bumper), NAV_ON_DEADEND() marks the most
 *     recent intersection's choice as failed, flips it to "go straight
 *     through instead", backs up, and turns the SAME side it originally
 *     turned. Since it just did a 180 first, turning the same side
 *     again nets out to the heading the robot had *before* that branch -
 *     i.e. it ends up continuing past the intersection in the original
 *     direction, which is what "try going straight instead" means. If
 *     going straight *also* dead-ends, both of this L/T junction's two
 *     possible choices have failed, which my project spec says
 *     shouldn't happen - I treat that as a hard stop rather than an
 *     infinite loop.
 *   - On the rear bumper (forward destination reached),
 *     NAV_ON_DESTINATION() does a 180 and switches CRNT_STATE to
 *     RETRACE, which drives back along the line and, at each
 *     intersection (in reverse order), takes the MIRROR of what I
 *     recorded there (straight stays straight; left becomes right and
 *     vice versa - the standard rule for reversing a turn-by-turn
 *     route). This is a different maneuver from the dead-end case
 *     above: there, the goal is to resume the original forward
 *     direction past one intersection; here, the goal is to actually
 *     retrace the whole path backward to the start.
 *   - HEADING (N/E/S/W, starting East per the spec) is updated
 *     alongside all of this purely for telemetry/debugging - none of
 *     my driving logic above actually depends on it.
 *
 *  Caveats I can only resolve by testing on the real robot:
 *   - T_UTURN (the pivot duration for a 180) is a guess, scaled from
 *     my existing ~90-degree branch-turn timing. I'll need to retune
 *     it on hardware, same as T_LEFT/T_RIGHT.
 *   - I'm assuming that re-detecting a branch/intersection while
 *     approaching it from the reverse direction (during RETRACE, or
 *     after backing out of a dead end) triggers the same BOW/MID
 *     off-line condition FWD_ST uses going forward. That's a
 *     reasonable assumption given the sensor layout but I haven't
 *     verified it against real sensor geometry.
 *   - MAX_INTERSECTIONS is set to 10 (my spec estimates "7 or so").
 *     I'll raise it if my maze has more junctions than that - beyond
 *     the limit, NAV_ENTER_BRANCH() stops recording new intersections
 *     but the robot keeps driving forward.
 *******************************************************************/
#include <hidef.h>            /* common defines and macros */
#include "derivative.h"       /* derivative-specific definitions */

/* LCD equates */
#define CLEAR_HOME    0x01    /* clear the display and home the cursor */
#define CURSOR_OFF    0x0C    /* display on, cursor off */
#define SHIFT_OFF     0x06    /* address increments, no character shift */
#define LCD_SEC_LINE  64      /* starting addr. of 2nd line of LCD */

#define LCD_CNTR      PTJ     /* LCD Control Register: E = PJ7, RS = PJ6 */
#define LCD_DAT       PORTB   /* LCD Data Register: D7 = PB7, ..., D0 = PB0 */
#define LCD_E         0x80
#define LCD_RS        0x40

#define DEL_50US_INNER_COUNT 300

/* Turn timers, in TOF ticks */
#define T_LEFT   8
#define T_RIGHT  8

/* Pivot duration for a ~180-degree U-turn (dead-end recovery / heading
   home from the forward destination). I scaled this from my ~90-degree
   branch commit delay (del_50us(6000) in FWD_ST) - I'll need to retune
   it on hardware. */
#define T_UTURN  12000

/* Robot states */
#define START        0
#define FWD          1
#define ALL_STOP     2
#define LEFT_TRN     3
#define RIGHT_TRN    4
#define LEFT_ALIGN   5
#define RIGHT_ALIGN  6
#define RETRACE      7

/* Compass heading - telemetry only, starts facing East per the spec.
   Turning right advances it, turning left goes back, mod 4. */
#define HEAD_NORTH 0
#define HEAD_EAST  1
#define HEAD_SOUTH 2
#define HEAD_WEST  3

/* A branch choice, as recorded per intersection */
#define NAV_STRAIGHT 0
#define NAV_LEFT     1
#define NAV_RIGHT    2

#define MAX_INTERSECTIONS 10

#define LINE_LEN 20

/* variables */

/* Baseline sensor readings and their allowed variance, established
   by testing against the actual guide track / robot. */
unsigned char BASE_LINE = 0x9D;
unsigned char BASE_BOW  = 0xCA;
unsigned char BASE_MID  = 0xCA;
unsigned char BASE_PORT = 0xCC;
unsigned char BASE_STBD = 0xCC;

unsigned char LINE_VARIANCE        = 0x18;
unsigned char BOW_VARIANCE         = 0x30;
unsigned char PORT_VARIANCE        = 0x20;
unsigned char MID_VARIANCE         = 0x20;
unsigned char STARBOARD_VARIANCE   = 0x15;

char TOP_LINE[LINE_LEN + 1];
char BOT_LINE[LINE_LEN + 1];
const char CLEAR_LINE[LINE_LEN + 1] = "                    ";

/* Guider sensor readings, initialized to my test values */
unsigned char SENSOR_LINE = 0x01;
unsigned char SENSOR_BOW  = 0x23;
unsigned char SENSOR_PORT = 0x45;
unsigned char SENSOR_MID  = 0x67;
unsigned char SENSOR_STBD = 0x89;
unsigned char SENSOR_NUM;

volatile unsigned char TOF_COUNTER = 0;
unsigned char CRNT_STATE = ALL_STOP;
unsigned char T_TURN;
unsigned char TEN_THOUS;
unsigned char THOUSANDS;
unsigned char HUNDREDS;
unsigned char TENS;
unsigned char UNITS;

/* My navigation manager: the maze-learning layer I added on top of
   FWD_ST's reactive line-following. See my file header for the design. */
unsigned char HEADING = HEAD_EAST;

unsigned char MAZE_CHOICE[MAX_INTERSECTIONS];   /* my recorded NAV_STRAIGHT/LEFT/RIGHT per intersection */
unsigned char MAZE_RETRIED[MAX_INTERSECTIONS];  /* has the alternate choice been tried here? */
unsigned char MAZE_COUNT = 0;                   /* intersections currently on record */

/* Set while backing out of a dead end to retry the other choice at
   MAZE_COUNT (left un-incremented, pointing at the intersection being
   redone). Lets NAV_ON_DEADEND() tell "the retry itself dead-ended"
   apart from "a brand-new intersection dead-ended", even if the retry
   fails before any branch sensor fires again (e.g. straight-ahead
   immediately hits a wall). */
unsigned char NAV_PENDING_RETRY = 0;
unsigned char NAV_PENDING_INDEX = 0;

/* How many recorded intersections are left to replay (in reverse,
   mirrored) while CRNT_STATE == RETRACE. */
unsigned char RETRACE_REMAINING = 0;

const char HEX_TABLE[16] = {
    '0', '1', '2', '3', '4', '5', '6', '7',
    '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'
};

/* messages */
const char msg1[] = "Battery volt ";
const char msg2[] = "State";
const char tab_start[]       = "start  ";
const char tab_fwd[]         = "fwd    ";
const char tab_all_stp[]     = "all_stp";
const char tab_left_turn[]   = "LeftTurn  ";
const char tab_right_turn[]  = "RightTurn  ";
const char tab_left_align[]  = "LeftAlign ";
const char tab_right_align[] = "RightAlign";
const char tab_retrace[]     = "Retrace";

const char * const tab[8] = {
    tab_start, tab_fwd, tab_all_stp, tab_left_turn,
    tab_right_turn, tab_left_align, tab_right_align, tab_retrace
};

/* prototypes */
void DISPATCHER(unsigned char state);
void START_ST(void);
void FWD_ST(void);
void LEFT(void);
void LEFT_ALIGN_DONE(void);
void RIGHT(void);
void RIGHT_ALIGN_DONE(void);
void ALL_STOP_ST(void);
void RETRACE_ST(void);

unsigned char NAV_APPLY_TURN(unsigned char heading, unsigned char side);
unsigned char NAV_OPPOSITE_SIDE(unsigned char side);
void NAV_RECORD_CHOICE(unsigned char side);
unsigned char NAV_ENTER_BRANCH(unsigned char side);
void NAV_ON_DEADEND(void);
void NAV_ON_DESTINATION(void);

void INIT_RIGHT(void);
void INIT_LEFT(void);
void INIT_FWD(void);
void INIT_REV(void);
void INIT_STOP(void);

void INIT(void);
void openADC(void);
void STRCPY(const char *src, char *dst);
void CLR_LCD_BUF(void);
void G_LEDS_ON(void);
void G_LEDS_OFF(void);
void READ_SENSORS(void);
void SELECT_SENSOR(unsigned char sensor_num);
void DISPLAY_SENSORS(void);

void initLCD(void);
void clrLCD(void);
void del_50us(unsigned int n);
void cmd2LCD(unsigned char cmd);
void putsLCD(const char *s);
void putcLCD(unsigned char c);
void dataMov(unsigned char b);
void initAD(void);
void LCD_POS_CRSR(unsigned char addr);

void int2BCD(unsigned int val);
void BCD2ASC(void);
void BIN2ASC(unsigned char val, char *hi, char *lo);

void ENABLE_TOF(void);
void UPDT_DISPL(void);

/*******************************************************************
 * Entry point
 *******************************************************************/
void main(void) {
    EnableInterrupts;               /* CLI */

    INIT();
    openADC();
    initLCD();
    CLR_LCD_BUF();

    DDRA |= 0x03;                   /* STAR_DIR, PORT_DIR */
    DDRT |= 0x30;                   /* STAR_SPEED, PORT_SPEED */

    initAD();                       /* final ADC config (overrides openADC's) */
    initLCD();
    clrLCD();

    putsLCD(msg1);

    cmd2LCD(0xC0);
    putsLCD(msg2);

    ENABLE_TOF();

    for (;;) {
        G_LEDS_ON();
        READ_SENSORS();
        G_LEDS_OFF();
        UPDT_DISPL();
        DISPATCHER(CRNT_STATE);
    }
}

/*******************************************************************
 * State dispatcher - exactly one state handler per call.
 *******************************************************************/
void DISPATCHER(unsigned char state) {
    switch (state) {
        case START:        START_ST();          break;
        case FWD:           FWD_ST();           break;
        case ALL_STOP:      ALL_STOP_ST();       break;
        case LEFT_TRN:       LEFT();             break;
        case RIGHT_TRN:      RIGHT();            break;
        case LEFT_ALIGN:     LEFT_ALIGN_DONE();  break;
        case RIGHT_ALIGN:    RIGHT_ALIGN_DONE(); break;
        case RETRACE:         RETRACE_ST();      break;
        default:
            break;
    }
}

/*******************************************************************/
void START_ST(void) {
    if (PORTAD0 & 0x04) {
        INIT_FWD();
        CRNT_STATE = FWD;
    }
}

/*******************************************************************
 * FORWARD state: line-following, intersection/turn decisions,
 * dead-end (bow bumper) and destination (rear bumper) detection.
 *******************************************************************/
void FWD_ST(void) {
    signed char diff;

    if ((PORTAD0 & 0x04) == 0) {
        /* bow bumper hit: dead end - hand off to the navigation manager
           to back out, mark this branch wrong, and try the alternate. */
        NAV_ON_DEADEND();
        return;
    }

    if ((PORTAD0 & 0x08) == 0) {
        /* rear bumper: operator signals forward destination reached -
           turn around and retrace the learned path back to the start. */
        NAV_ON_DESTINATION();
        return;
    }

    diff = (signed char)(SENSOR_BOW + BOW_VARIANCE - BASE_BOW);
    if (diff >= 0) goto NOT_ALIGNED;

    diff = (signed char)(SENSOR_MID + MID_VARIANCE - BASE_MID);
    if (diff >= 0) goto NOT_ALIGNED;

    diff = (signed char)(SENSOR_LINE + LINE_VARIANCE - BASE_LINE);
    if (diff >= 0) goto CHECK_RIGHT_ALIGN;

    diff = (signed char)(SENSOR_LINE - LINE_VARIANCE - BASE_LINE);
    if (diff < 0) goto CHECK_LEFT_ALIGN;

NOT_ALIGNED:
    diff = (signed char)(SENSOR_PORT + PORT_VARIANCE - BASE_PORT);
    if (diff >= 0) goto PARTIAL_LEFT_TRN;

    diff = (signed char)(SENSOR_BOW + BOW_VARIANCE - BASE_BOW);
    if (diff >= 0) return;

    diff = (signed char)(SENSOR_STBD + STARBOARD_VARIANCE - BASE_STBD);
    if (diff >= 0) goto PARTIAL_RIGHT_TRN;
    return;

PARTIAL_LEFT_TRN:
    if (!NAV_ENTER_BRANCH(NAV_LEFT)) return;   /* recorded choice: go straight */
    del_50us(6000);
    INIT_LEFT();
    CRNT_STATE = LEFT_TRN;
    del_50us(6000);
    return;

CHECK_LEFT_ALIGN:
    INIT_LEFT();
    CRNT_STATE = LEFT_ALIGN;
    return;

PARTIAL_RIGHT_TRN:
    if (!NAV_ENTER_BRANCH(NAV_RIGHT)) return;  /* recorded choice: go straight */
    del_50us(6000);
    INIT_RIGHT();
    CRNT_STATE = RIGHT_TRN;
    del_50us(6000);
    return;

CHECK_RIGHT_ALIGN:
    INIT_RIGHT();
    CRNT_STATE = RIGHT_ALIGN;
    return;
}

/*******************************************************************/
void LEFT(void) {
    signed char diff = (signed char)(SENSOR_BOW + BOW_VARIANCE - BASE_BOW);
    if (diff >= 0) {
        LEFT_ALIGN_DONE();
    }
}

void LEFT_ALIGN_DONE(void) {
    CRNT_STATE = FWD;
    INIT_FWD();
}

void RIGHT(void) {
    signed char diff = (signed char)(SENSOR_BOW + BOW_VARIANCE - BASE_BOW);
    if (diff >= 0) {
        RIGHT_ALIGN_DONE();
    }
}

void RIGHT_ALIGN_DONE(void) {
    CRNT_STATE = FWD;
    INIT_FWD();
}

/*******************************************************************/
void ALL_STOP_ST(void) {
    if ((PORTAD0 & 0x04) == 0) {
        CRNT_STATE = START;
    }
}

/*******************************************************************
 * Navigation manager: intersection memory, dead-end backtracking, and
 * retracing the learned path home. See my file header for the design.
 *******************************************************************/

/* heading after turning `side` from `heading` (NAV_LEFT/NAV_RIGHT/
   NAV_STRAIGHT); clockwise = right, per the spec's convention. */
unsigned char NAV_APPLY_TURN(unsigned char heading, unsigned char side) {
    if (side == NAV_RIGHT) return (unsigned char)((heading + 1) & 0x03);
    if (side == NAV_LEFT)  return (unsigned char)((heading + 3) & 0x03);
    return heading;
}

unsigned char NAV_OPPOSITE_SIDE(unsigned char side) {
    if (side == NAV_LEFT)  return NAV_RIGHT;
    if (side == NAV_RIGHT) return NAV_LEFT;
    return NAV_STRAIGHT;
}

/* Records a fresh branch choice at the current intersection (MAZE_COUNT)
   and advances HEADING/MAZE_COUNT. */
void NAV_RECORD_CHOICE(unsigned char side) {
    if (MAZE_COUNT < MAX_INTERSECTIONS) {
        MAZE_CHOICE[MAZE_COUNT] = side;
        MAZE_RETRIED[MAZE_COUNT] = 0;
        MAZE_COUNT++;
    }
    HEADING = NAV_APPLY_TURN(HEADING, side);
}

/* Called from FWD_ST whenever a branch is detected, before the turn is
   committed. Returns 1 if the caller should take the branch (turn), or
   0 if it should be ignored (keep driving straight) because a prior
   dead-end retry already recorded "go straight" here. */
unsigned char NAV_ENTER_BRANCH(unsigned char side) {
    if (NAV_PENDING_RETRY && MAZE_COUNT == NAV_PENDING_INDEX) {
        NAV_PENDING_RETRY = 0;
        MAZE_COUNT++;            /* successfully passed this intersection */
        return 0;
    }
    NAV_RECORD_CHOICE(side);
    return 1;
}

/* Called from FWD_ST on a front-bumper (dead-end) hit. */
void NAV_ON_DEADEND(void) {
    unsigned char idx;
    unsigned char failed_side;

    UPDT_DISPL();

    if (NAV_PENDING_RETRY) {
        /* The alternate ("straight through") choice at this
           intersection also dead-ends - both of this L/T junction's
           choices have failed, which shouldn't happen per the project
           spec. Stop rather than loop forever. */
        NAV_PENDING_RETRY = 0;
        CRNT_STATE = ALL_STOP;
        INIT_STOP();
        return;
    }

    if (MAZE_COUNT == 0) {
        /* No intersection to blame yet - simple bump recovery, keep
           exploring in the same direction. */
        INIT_REV();
        del_50us(6000);
        INIT_RIGHT();
        del_50us(6000);
        INIT_FWD();
        CRNT_STATE = FWD;
        return;
    }

    idx = (unsigned char)(MAZE_COUNT - 1);
    failed_side = MAZE_CHOICE[idx];

    MAZE_RETRIED[idx] = 1;
    MAZE_CHOICE[idx] = NAV_STRAIGHT;   /* try going straight through next */
    MAZE_COUNT = idx;                  /* un-count it - we're redoing it */

    NAV_PENDING_RETRY = 1;
    NAV_PENDING_INDEX = idx;

    /* Undo the failed turn: heading is currently "after the branch";
       turning the same side again after a 180 nets out to "before the
       branch", i.e. the original forward direction past this
       intersection. */
    HEADING = NAV_APPLY_TURN(HEADING, NAV_OPPOSITE_SIDE(failed_side));

    INIT_REV();
    del_50us(6000);                    /* back out of the dead-end branch */
    if (failed_side == NAV_RIGHT) {
        INIT_RIGHT();
    } else {
        INIT_LEFT();
    }
    del_50us(T_UTURN);                 /* pivot back onto the main corridor */
    INIT_FWD();
    CRNT_STATE = FWD;
}

/* Called from FWD_ST on a rear-bumper hit (forward destination). */
void NAV_ON_DESTINATION(void) {
    INIT_STOP();
    UPDT_DISPL();

    HEADING = (unsigned char)((HEADING + 2) & 0x03);   /* about-face */

    INIT_REV();
    del_50us(6000);
    INIT_RIGHT();                      /* either side works for a plain 180 */
    del_50us(T_UTURN);
    INIT_FWD();

    RETRACE_REMAINING = MAZE_COUNT;
    CRNT_STATE = RETRACE;
}

/*******************************************************************
 * RETRACE state: drive back along the line toward the start, replaying
 * the recorded intersections in reverse with mirrored turns (straight
 * stays straight; left <-> right). Reaching the start is detected the
 * same way a dead end is (front bumper) - by then RETRACE_REMAINING is
 * 0, so it's treated as "arrived home" rather than a failed branch.
 *******************************************************************/
void RETRACE_ST(void) {
    signed char diff;
    unsigned char idx;
    unsigned char mirrored;

    if ((PORTAD0 & 0x04) == 0) {
        /* Back at the start - done. */
        INIT_STOP();
        CRNT_STATE = ALL_STOP;
        return;
    }

    diff = (signed char)(SENSOR_BOW + BOW_VARIANCE - BASE_BOW);
    if (diff >= 0) goto CHECK_BRANCH;
    diff = (signed char)(SENSOR_MID + MID_VARIANCE - BASE_MID);
    if (diff >= 0) goto CHECK_BRANCH;
    return;                             /* still on the line, no branch here */

CHECK_BRANCH:
    if (RETRACE_REMAINING == 0) return; /* no more recorded turns - keep straight */

    idx = (unsigned char)(RETRACE_REMAINING - 1);
    RETRACE_REMAINING = idx;

    if (MAZE_CHOICE[idx] == NAV_STRAIGHT) return;

    mirrored = NAV_OPPOSITE_SIDE(MAZE_CHOICE[idx]);
    HEADING = NAV_APPLY_TURN(HEADING, mirrored);

    del_50us(6000);
    if (mirrored == NAV_RIGHT) {
        INIT_RIGHT();
    } else {
        INIT_LEFT();
    }
    del_50us(6000);
    INIT_FWD();
}

/*******************************************************************
 * Initialization subroutines
 *******************************************************************/
void INIT_RIGHT(void) {
    PORTA |= 0x02;
    PORTA &= (unsigned char)~0x01;
    T_TURN = TOF_COUNTER + T_RIGHT;
}

void INIT_LEFT(void) {
    PORTA |= 0x01;
    PORTA &= (unsigned char)~0x02;
    T_TURN = TOF_COUNTER + T_LEFT;
}

void INIT_FWD(void) {
    PORTA &= (unsigned char)~0x03;  /* FWD direction, both motors */
    PTT |= 0x30;                    /* drive motors on */
}

void INIT_REV(void) {
    PORTA |= 0x03;                  /* REV direction, both motors */
    PTT |= 0x30;                    /* drive motors on */
}

void INIT_STOP(void) {
    PTT &= (unsigned char)~0x30;    /* drive motors off */
}

/*******************************************************************
 * Initialize sensors/ports: PORTAD input, PORTA/PORTB/PORTJ(7:6) out
 *******************************************************************/
void INIT(void) {
    DDRAD = 0x00;                    /* PORTAD all input */
    DDRA |= 0xFF;
    DDRB |= 0xFF;
    DDRJ |= 0xC0;
}

/*******************************************************************
 * Initialize ADC (first pass - see initAD() for the config that
 * actually ends up in effect).
 *******************************************************************/
void openADC(void) {
    ATDCTL2 = 0x80;                 /* turn on ADC */
    del_50us(1);                    /* wait for ADC to be ready */
    ATDCTL3 = 0x20;                 /* 4 conversions on channel AN1 */
    ATDCTL4 = 0x97;                 /* 8-bit resolution, prescaler=48 */
}

/*******************************************************************
 * Copies a null-terminated string, including the null.
 *******************************************************************/
void STRCPY(const char *src, char *dst) {
    for (;;) {
        *dst = *src;
        if (*dst == '\0') break;
        src++;
        dst++;
    }
}

/*******************************************************************
 * Fill the LCD display buffers with spaces, ready to build a
 * new display buffer.
 *******************************************************************/
void CLR_LCD_BUF(void) {
    STRCPY(CLEAR_LINE, TOP_LINE);
    STRCPY(CLEAR_LINE, BOT_LINE);
}

/*******************************************************************
 * Guider LEDs ON/OFF: readings correspond to "illuminated" vs
 * "ambient lighting" when read with the LEDs off.
 *******************************************************************/
void G_LEDS_ON(void) {
    PORTA |= 0x20;
}

void G_LEDS_OFF(void) {
    PORTA &= (unsigned char)~0x20;
}

/*******************************************************************
 * Read all 5 guider sensors (line, bow, port, mid, starboard) in
 * turn, selecting each on the analog mux before converting.
 *******************************************************************/
void READ_SENSORS(void) {
    static unsigned char * const sensor_reg[5] = {
        &SENSOR_LINE, &SENSOR_BOW, &SENSOR_PORT, &SENSOR_MID, &SENSOR_STBD
    };
    unsigned char i;

    for (i = 0; i < 5; i++) {
        SENSOR_NUM = i;
        SELECT_SENSOR(i);
        del_50us(400);               /* let the sensor stabilize */

        ATDCTL5 = 0x81;               /* start A/D conversion on AN1 */
        while ((ATDSTAT0 & 0x80) == 0) {
            /* wait for conversion to complete */
        }

        /* ATDCTL3 configures 4 conversions per sequence; average all 4
           (into ATDDR0L..ATDDR3L) instead of just using ATDDR0L, to
           reduce noise on the sensor reading. */
        *sensor_reg[i] = (unsigned char)
            (((unsigned int)ATDDR0L + ATDDR1L + ATDDR2L + ATDDR3L) / 4);
    }
}

/*******************************************************************
 * Select which sensor's signal is routed to the ADC input, via the
 * analog mux select bits on PORTA (bits 2-4).
 *******************************************************************/
void SELECT_SENSOR(unsigned char sensor_num) {
    unsigned char temp = (unsigned char)(PORTA & 0xE3);
    unsigned char sel  = (unsigned char)((sensor_num << 2) & 0x1C);
    PORTA = (unsigned char)(sel | temp);
}

/*******************************************************************
 * Debug readout of the raw sensor values on the LCD. I don't call
 * it from main()'s loop by default - I can call it instead of/
 * alongside UPDT_DISPL() if I want the sensor readout my project
 * write-up recommends.
 *******************************************************************/
void DISPLAY_SENSORS(void) {
    BIN2ASC(SENSOR_BOW,  &TOP_LINE[3], &TOP_LINE[4]);
    BIN2ASC(SENSOR_PORT, &BOT_LINE[0], &BOT_LINE[1]);
    BIN2ASC(SENSOR_MID,  &BOT_LINE[3], &BOT_LINE[4]);
    BIN2ASC(SENSOR_STBD, &BOT_LINE[6], &BOT_LINE[7]);
    BIN2ASC(SENSOR_LINE, &BOT_LINE[9], &BOT_LINE[10]);

    cmd2LCD(CLEAR_HOME);
    del_50us(40);

    putsLCD(TOP_LINE);
    LCD_POS_CRSR(LCD_SEC_LINE);
    putsLCD(BOT_LINE);
}

/*******************************************************************
 * Initialization of the LCD: 4-bit data width, 2-line display,
 * turn on display, cursor and blinking off. Shift cursor right.
 *******************************************************************/
void initLCD(void) {
    DDRB |= 0xFF;
    DDRJ |= 0xC0;

    del_50us(2000);

    cmd2LCD(0x28);
    cmd2LCD(0x0C);
    cmd2LCD(0x06);
}

/*******************************************************************
 * Clear display and home cursor
 *******************************************************************/
void clrLCD(void) {
    cmd2LCD(0x01);
    del_50us(40);
}

/*******************************************************************
 * Approximately (n x 50us) busy-wait delay.
 *******************************************************************/
void del_50us(unsigned int n) {
    unsigned int i;
    volatile unsigned int j;

    for (i = 0; i < n; i++) {
        for (j = 0; j < DEL_50US_INNER_COUNT; j++) {
            /* spin */
        }
    }
}

/*******************************************************************
 * Sends a command to the LCD instruction register
 *******************************************************************/
void cmd2LCD(unsigned char cmd) {
    LCD_CNTR &= (unsigned char)~LCD_RS;
    dataMov(cmd);
}

/*******************************************************************
 * Outputs a NULL-terminated string
 *******************************************************************/
void putsLCD(const char *s) {
    while (*s) {
        putcLCD((unsigned char)*s);
        s++;
    }
}

/*******************************************************************
 * Outputs one character to the LCD
 *******************************************************************/
void putcLCD(unsigned char c) {
    LCD_CNTR |= LCD_RS;
    dataMov(c);
}

/*******************************************************************
 * Sends a byte to the LCD IR or DR, as two 4-bit nibble writes.
 *******************************************************************/
void dataMov(unsigned char b) {
    LCD_CNTR |= LCD_E;
    LCD_DAT = b;
    LCD_CNTR &= (unsigned char)~LCD_E;

    b = (unsigned char)(b << 4);

    LCD_CNTR |= LCD_E;
    LCD_DAT = b;
    LCD_CNTR &= (unsigned char)~LCD_E;

    del_50us(1);
}

/*******************************************************************/
void initAD(void) {
    ATDCTL2 = 0xC0;
    del_50us(1);
    ATDCTL3 = 0x00;
    ATDCTL4 = 0x85;
    ATDDIEN |= 0x0C;
}

/*******************************************************************
 * Positions the LCD cursor at the given DDRAM address.
 *******************************************************************/
void LCD_POS_CRSR(unsigned char addr) {
    cmd2LCD((unsigned char)(addr | 0x80));
}

/*****************************************************************
 * Integer to BCD Conversion Routine
 *****************************************************************/
void int2BCD(unsigned int val) {
    TEN_THOUS = 0;
    THOUSANDS = 0;
    HUNDREDS  = 0;
    TENS      = 0;
    UNITS     = 0;

    if (val == 0) return;

    UNITS = (unsigned char)(val % 10);
    val /= 10;
    if (val == 0) return;

    TENS = (unsigned char)(val % 10);
    val /= 10;
    if (val == 0) return;

    HUNDREDS = (unsigned char)(val % 10);
    val /= 10;
    if (val == 0) return;

    THOUSANDS = (unsigned char)(val % 10);
    val /= 10;
    if (val == 0) return;

    TEN_THOUS = (unsigned char)(val % 10);
}

/****************************************************************
 * BCD to ASCII Conversion Routine, with leading-zero blanking.
 ****************************************************************/
void BCD2ASC(void) {
    unsigned char no_blank = 0;

    if (TEN_THOUS != 0 || no_blank != 0) {
        TEN_THOUS |= 0x30;
        no_blank = 1;
    } else {
        TEN_THOUS = ' ';
    }

    if (THOUSANDS != 0 || no_blank != 0) {
        THOUSANDS |= 0x30;
        no_blank = 1;
    } else {
        THOUSANDS = ' ';
    }

    if (HUNDREDS != 0 || no_blank != 0) {
        HUNDREDS |= 0x30;
        no_blank = 1;
    } else {
        HUNDREDS = ' ';
    }

    if (TENS != 0 || no_blank != 0) {
        TENS |= 0x30;
        no_blank = 1;
    } else {
        TENS = ' ';
    }

    UNITS |= 0x30;
}

/*******************************************************************
 * Converts a byte into two ASCII hex digit characters.
 *******************************************************************/
void BIN2ASC(unsigned char val, char *hi, char *lo) {
    *lo = HEX_TABLE[val & 0x0F];
    *hi = HEX_TABLE[(unsigned char)(val >> 4) & 0x0F];
}

/************************************************************/
void ENABLE_TOF(void) {
    TSCR1 = 0x80;
    TFLG2 = 0x80;
    TSCR2 = 0x84;
}

/************************************************************
 * Timer overflow ISR. The VectorNumber_Vtimovf macro comes from
 * derivative.h; if my CodeWarrior version names the timer-overflow
 * vector differently, I need to search that header for "timovf"
 * and adjust.
 ************************************************************/
void interrupt VectorNumber_Vtimovf TOF_ISR(void) {
    TOF_COUNTER++;
    TFLG2 = 0x80;
}

/*******************************************************************
 * Update Display (Battery Voltage + Current State)
 *******************************************************************/
void UPDT_DISPL(void) {
    unsigned int battery;

    ATDCTL5 = 0x90;
    while ((ATDSTAT0 & 0x80) == 0) {
        /* wait until the conversion sequence is complete */
    }
    battery = ATDDR0L;

    battery = (unsigned int)(battery * 39) + 600;
    int2BCD(battery);
    BCD2ASC();

    cmd2LCD(0x8D);                  /* 1st row, end of msg1 */
    putcLCD(TEN_THOUS);
    putcLCD(THOUSANDS);
    putcLCD('.');
    putcLCD(HUNDREDS);

    cmd2LCD(0xC7);                  /* 2nd row, end of msg2 (as in source) */
    putsLCD(tab[CRNT_STATE]);
}
