/* -*- mode: c++; -*- */

#include "CDS.h"
#include "DHT.h"
#include "KappaGPS.h"
#include "KappaDateTime.h"
#include "KappaSegment.h"
#include "Button.h"
#include "NoInteraction.h"

enum COM_POSITION {
    COM_POSITION_HOUR        = 0,
    COM_POSITION_MINUTE      = 1,
    COM_POSITION_SECOND      = 2,
    COM_POSITION_YEAR_H      = 3,
    COM_POSITION_YEAR_L      = 4,
    COM_POSITION_MONTH       = 5,
    COM_POSITION_DAY         = 6,
    COM_POSITION_TEMPERATURE = 7,
    COM_POSITION_LUNAR_MONTH = 8,
    COM_POSITION_LUNAR_DAY   = 9,
};

typedef struct {
    volatile uint8_t* port;
    uint8_t pin;
} LED_COMSEG_T;

typedef enum {
    RESET_STATE,
    FACTORYTEST_STATE,
    STARTED_STATE,
    NORMAL_STATE,
    TIMESETTING_STATE,
} State;

const uint8_t COMS_ARDUINO[] = {
    32,                         /* COM1 */
    33,                         /* COM2 */
    36                          /* COM11 */
};

const uint8_t COMS_DECIMAL_PLACES[] = {
    0,                          // COM1
    1,                          // COM2
};

const uint8_t COMS_BRIGHTNESS[3][2] = {
    // low (0)
    {
        8,                      // COM1
        8,                      // COM2
    },
    // medium (1)
    {
        60,                     // COM1
        60,                     // COM2
    },
    // high (2)
    {
        180,                    // COM1
        180,                    // COM2
    },
};

const uint8_t SEGS_ARDUINO[] = {
    44,                         // SEG1
    43,                         // SEG2
    42,                         // SEG3
    41,                         // SEG4
    40,                         // SEG5
    39,                         // SEG6
    38,                         // SEG7
    37,                         // SEG8

    8,                          // SEG9
    12,                         // SEG10
    13,                         // SEG11
    14,                         // SEG12
    28,                         // SEG13
    29,                         // SEG14
    30,                         // SEG15
    31                          // SEG16
};


const uint8_t NUM_TO_SEG[] = {
    //76543210
    0b01110111,                 // 0
    0b00010010,                 // 1
    0b01101011,                 // 2
    0b01011011,                 // 3
    0b00011110,                 // 4
    0b01011101,                 // 5
    0b01111101,                 // 6
    0b00010111,                 // 7
    0b01111111,                 // 8
    0b01011111,                 // 9
    0b00000000,                 // NULL
};

void LED_COM_ON(uint8_t com_number)
{
    //digitalWrite(34, HIGH);
    digitalWrite(COMS_ARDUINO[com_number], HIGH);
};

void LED_COM_OFF(uint8_t com_number)
{
    if (com_number == 0)
        digitalWrite(34, LOW);
    digitalWrite(COMS_ARDUINO[com_number], LOW);
}

void LED_COM_ALL_OFF()
{
    for (int i = 0; i < 3; i++)
        LED_COM_OFF(i);
}

void LED_COM_BRIGHTNESS(uint8_t com_number, uint8_t level)
{
    /* analogWrite(15, COMS_BRIGHTNESS[level][com_number]); */
    OCR2 = COMS_BRIGHTNESS[level][com_number];
}

void LED_SEG_SET_VALUE(uint8_t value1, uint8_t value2)
{
    uint8_t v1 = NUM_TO_SEG[value1];
    uint8_t v2 = NUM_TO_SEG[value2];

    digitalWrite(34, HIGH);
    for (int i = 0; i < 7; i++)	{
        digitalWrite(SEGS_ARDUINO[i], (v1 & (1 << i)) ? HIGH : LOW);
        digitalWrite(SEGS_ARDUINO[i + 8], (v2 & (1 << i)) ? HIGH : LOW);
    }
}

void LED_SET_NULL()
{
    digitalWrite(34, LOW);
    for (int i = 0; i < 7; i++) {
        digitalWrite(SEGS_ARDUINO[i], LOW);
        digitalWrite(SEGS_ARDUINO[i + 8], LOW);
    }
}

static uint8_t bcdToDec(uint8_t val)
{
    return ((val >> 4) * 10) + (val % 16);
}

static uint8_t decToBcd(uint8_t val)
{
    return ((val / 10 * 16) + (val % 10));
}


void LED_DISPLAY_VALUE(uint8_t decimal_val, uint8_t com_number)
{
    if (com_number == 0 && decimal_val > 12) {
        decimal_val -= 12;
    }

    uint8_t bcd_val = decToBcd(decimal_val);

    for (int i = 0; i < 7; i++)	{
        if (((NUM_TO_SEG[bcd_val >> 4] & (1 << (i))) == 0) ||
            (decimal_val < 10 && COMS_DECIMAL_PLACES[com_number] == 0))	{
            digitalWrite(SEGS_ARDUINO[i], LOW);
        }
        else {
            if (com_number == 0)
                digitalWrite(34, HIGH);
            else
                digitalWrite(SEGS_ARDUINO[i], HIGH);
        }

    }

    for (int i = 0; i < 7; i++) {
        if ((NUM_TO_SEG[bcd_val & 0x0F] & (1 << (i))) == 0) {
            digitalWrite(SEGS_ARDUINO[i + 8], LOW);
        }
        else {
            digitalWrite(SEGS_ARDUINO[i + 8], HIGH);
        }
    }
}


// PIN 15 = PB7 = PWM (밝기 조절)
// PIN 18 = PD0 = I2C SCL
// PIN 19 = PD1 = I2C SDA
// PIN 20 = PD2 = GPS UART RX (9600bps)

static KappaGPS gps;
static KappaDateTime datetime;
static KappaSegment matrix;
static CDS cds;
static DHT dht;
static Button btn_reset(49);
static Button btn_select(50);
static Button btn_up(51);
static Button btn_down(52);
static NoInteraction nointeraction;

volatile uint8_t cn = 0;
volatile uint8_t cn_1 = 0, cn_2 = 0;
volatile uint8_t *digits = NULL;
volatile uint8_t year_upper = 20;
volatile uint8_t com11_segs = 0;

bool time_synced_between_gps_n_rtc = false;

static bool blink_flag = false;

State state = STARTED_STATE;


static uint8_t setting_com_sequence[] = {
    COM_POSITION_YEAR_L,
    COM_POSITION_MONTH,
    COM_POSITION_DAY,
    COM_POSITION_HOUR,
    COM_POSITION_MINUTE,
    0xFF
};
static uint8_t *current_setting_com = &setting_com_sequence[0];
#define SETTING_COM_POSITION_INIT()   (current_setting_com = &setting_com_sequence[0])

// Restarts this program from beginning but does not reset the
// peripherals and registers
void reset()
{
    state = RESET_STATE;
    matrix.turnOffAll();
    cds.setBrightness(2);
    gps.stop();
    gps.start();

    delay(500);
    asm volatile ("  jmp 0");
}

static void onNotifyRTC(void *arg)
{
    KappaDateTime *dt = (KappaDateTime *) arg;

    if (dt) {

        Rtc::DateTime *c = dt->current();

        if (dt->isSettingMode()) {
            blink_flag = !blink_flag;

            matrix.setBrightness(cds.getBrightness());
            matrix.setNumber(0, c->sec);

            if (blink_flag) {
                matrix.setNumber(1, year_upper);
                matrix.setNumber(2, c->year);
                matrix.setNumber(3, c->mon);
                matrix.setNumber(4, c->day);
            } else {
                int com = *current_setting_com;
                if (com == COM_POSITION_YEAR_L) {
                    matrix.setTurnOff(1);
                    matrix.setTurnOff(2);
                } else if (com == COM_POSITION_MONTH) {
                    matrix.setTurnOff(3);
                } else if (com == COM_POSITION_DAY) {
                    matrix.setTurnOff(4);
                }
            }

            matrix.setNumber(5, (uint8_t) dht.getTemperature());
            matrix.setNumber(6, *(dt->lunarDate() + 1));
            matrix.setNumber(7, *(dt->lunarDate() + 2));
            matrix.write();
        }
        else {
            matrix.setBrightness(cds.getBrightness());
            matrix.setNumber(0, c->sec);
            matrix.setNumber(1, year_upper);
            matrix.setNumber(2, c->year);
            matrix.setNumber(3, c->mon);
            matrix.setNumber(4, c->day);
            matrix.setNumber(5, (uint8_t) dht.getTemperature());
            matrix.setNumber(6, *(dt->lunarDate() + 1));
            matrix.setNumber(7, *(dt->lunarDate() + 2));
            matrix.write();
        }
        //dt->printCurrent();
    }
}

static void onButtonReset(void *arg, Button::EVENT e)
{
    reset();
}

static void onButtonSelect(void *arg, Button::EVENT e)
{
    if (state == NORMAL_STATE) {

        if (e == Button::UP) {
            SETTING_COM_POSITION_INIT();
            blink_flag = false;
            gps.stop();
            datetime.changeToSettingMode();
            state = TIMESETTING_STATE;
            nointeraction.reset();
            Serial.println("state: NORMAL ==> TIMESETTING");
        }
    }
    else if (state == TIMESETTING_STATE) {
        nointeraction.reset();

        if (e == Button::UP) {
            if (*(++current_setting_com) == 0xFF) {
                SETTING_COM_POSITION_INIT();
                datetime.saveSettingTime();
                gps.transitToMiddleTerm();
                gps.start();
                datetime.changeToNormalMode();
                nointeraction.stop();
                state = NORMAL_STATE;
                time_synced_between_gps_n_rtc = false;
                Serial.println("state: TIMESETTING ==> NORMAL ");
            }
        }
    }
}

static void onButtonUp(void *arg, Button::EVENT e)
{
    if (state == TIMESETTING_STATE) {

        nointeraction.reset();

        uint8_t com = *current_setting_com;

        switch (com) {

        case COM_POSITION_HOUR:
            datetime.increaseHour();
            break;

        case COM_POSITION_MINUTE:
            datetime.increaseMinute();
            break;

        case COM_POSITION_YEAR_L:
            datetime.increaseYear();
            break;

        case COM_POSITION_MONTH:
            datetime.increaseMonth();
            break;

        case COM_POSITION_DAY:
            datetime.increaseDay();
            break;

        default:
            return;
        }
    }
}

static void onButtonDown(void *arg, Button::EVENT e)
{
    if (state == TIMESETTING_STATE) {

        nointeraction.reset();

        uint8_t com = *current_setting_com;

        switch (com) {
        case COM_POSITION_HOUR:
            datetime.decreaseHour();
            break;

        case COM_POSITION_MINUTE:
            datetime.decreaseMinute();
            break;

        case COM_POSITION_YEAR_L:
            datetime.decreaseYear();
            break;

        case COM_POSITION_MONTH:
            datetime.decreaseMonth();
            break;

        case COM_POSITION_DAY:
            datetime.decreaseDay();
            break;

        default:
            return;
        }
    }
}

static void onNoInteraction(void *arg)
{
    if (state == TIMESETTING_STATE) {
        SETTING_COM_POSITION_INIT();
        //Rtc::DateTime *c = datetime.current();
        //datetime.setCurrent(c);
        //time_synced_between_gps_n_rtc = false;
        datetime.changeToNormalMode();
        state = NORMAL_STATE;
        Serial.println("state: TIMESETTING ==> NORMAL ");
    }
}

static void onNotifyGPS(void *arg, bool ret)
{
    KappaGPS *obj = (KappaGPS *) arg;

    if (ret) {
        /* Sync with RTC */
        time_synced_between_gps_n_rtc = datetime.setCurrent(obj->getGPS());
        obj->transitToLongTerm();
    }
}

void setup()
{
    gps.init(onNotifyGPS);

    matrix.setup();

    // SETUP GPIO
    pinMode(34, OUTPUT); // Decimal place of COM1
    for (int i = 0; i < 3; i++) {
        pinMode(COMS_ARDUINO[i], OUTPUT);
    }

    for (int i=0; i<16; i++) {
        pinMode(SEGS_ARDUINO[i], OUTPUT);
    }

    pinMode(15, OUTPUT);
    //setPwmFrequency(15, 1024);
    digitalWrite(15, HIGH);		// 일단 밝기 조절은 넘어가고... 그냥 항상 ON 시켜놓음

    // OFF All COMs
    LED_COM_ALL_OFF();

    // initialize timer1
    noInterrupts();

    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = 0;
    /* OCR1A = 12;            // compare match register 16MHz/256/2Hz */
    OCR1A = 24;            // compare match register 16MHz/256/2Hz
    /* OCR1A = 31250;            // compare match register 16MHz/256/2Hz */
    TCCR1B |= (1 << WGM12);   // CTC mode
    TCCR1B |= (1 << CS12);    // 256 prescaler
    TIMSK |= (1 << OCIE1A);  // enable timer compare interrupt

    /* TCNT1 = 63974;   // for 1 sec at 16 MHz	 */

    /* TCCR1A = 0x00; */
    /* TCCR1B = (1<<CS10) | (1<<CS12);;  // Timer mode with 1024 prescler */
    /* TIMSK = (1 << TOIE1) ; // Enable timer1 overflow interrupt(TOIE1) */

    /* Timer2 */
    /* OCR2 = 62;  */
    /* TCCR2 = 0x28; */
    /* TCCR2 |= 0x03; */
    /* OCIE2_bit = 1; */
    OCR2 = 100;
    TCCR2 = 0x61;
    /* TIMSK |= (1 << OCIE2);  // enable timer compare interrupt */

    interrupts();

    datetime.init(onNotifyRTC);
    datetime.start(200);

    cds.start(500);
    dht.start(2000);

    btn_reset.registerCallback(onButtonReset);
    btn_select.registerCallback(onButtonSelect);
    btn_up.registerCallback(onButtonUp);
    btn_down.registerCallback(onButtonDown);

    btn_reset.start(30, false);
    btn_select.start(30, false);
    btn_up.start(30, false);
    btn_down.start(30, false);

    nointeraction.registerCallback(onNoInteraction);

    Serial.begin(115200);
    Serial.println("Setup Done.");

    /* up & down 버튼이 눌린 상태로 전원이 인가되면 factory test mode 로 진입함. */
    if (btn_down.isPushing() && btn_up.isPushing()) {
        state = FACTORYTEST_STATE;
        Serial.println("state: FACTORYTEST");
    }
}

ISR(TIMER1_COMPA_vect){         //timer0 interrupt

    if (state == STARTED_STATE || state == RESET_STATE) {
        cn_1 = cn / 2;
        cn_2 = cn % 2;

        LED_COM_ALL_OFF();
        LED_COM_ON(cn_1);
        LED_SET_NULL();

        if (++cn > 6)
            cn = 0;

        return;
    }
    else if (state == FACTORYTEST_STATE) {
        Rtc::DateTime *c = datetime.current();

        cn_1 = cn / 2;
        cn_2 = cn % 2;

        LED_COM_ALL_OFF();
        LED_COM_ON(cn_1);

        if (cn_2 == 0) {
            if (cn_1 == 0)       digits = &c->hour;
            else if (cn_1 == 1)  digits = &c->min;
            else if (cn_1 == 2) goto HANDLE_COM11_SEGS;
            else digits = nullptr;
        }
        else {
            digits = nullptr;
        }

        if (digits)	{
            LED_COM_BRIGHTNESS(cn_1, cds.getBrightness());
            LED_SEG_SET_VALUE(*digits, *digits);
        }
        else {
            LED_SET_NULL();
        }

        if (++cn > 6)
            cn = 0;

        return;
    }
    else if (state == NORMAL_STATE) {
        Rtc::DateTime *c = datetime.current();

        cn_1 = cn / 2;
        cn_2 = cn % 2;

        LED_COM_ALL_OFF();
        LED_COM_ON(cn_1);

        if (cn_2 == 0) {
            if (cn_1 == 0)       digits = &c->hour;
            else if (cn_1 == 1)  digits = &c->min;
            else if (cn_1 == 2) goto HANDLE_COM11_SEGS;
            else digits = nullptr;
        }
        else {
            digits = nullptr;
        }

        if (digits)	{
            LED_COM_BRIGHTNESS(cn_1, cds.getBrightness());
            if (cn_1 == 0 && *digits == 0)
                LED_DISPLAY_VALUE(12, 0);
            else
                LED_DISPLAY_VALUE(*digits, cn_1);
        }
        else {
            LED_SET_NULL();
        }

        if (++cn > 6)
            cn = 0;

        return;
    }
    else if (state == TIMESETTING_STATE) {
        Rtc::DateTime *c = datetime.current();

        cn_1 = cn / 2;
        cn_2 = cn % 2;
        LED_COM_ALL_OFF();
        LED_COM_ON(cn_1);

        if (cn_2 == 0) {
            if (cn_1 == 0)       digits = (*current_setting_com == 0) ? (blink_flag ? &c->hour : nullptr) : &c->hour;
            else if (cn_1 == 1)  digits = (*current_setting_com == 1) ? (blink_flag ? &c->min : nullptr) : &c->min;
            else if (cn_1 == 2) goto HANDLE_COM11_SEGS;
            else digits = nullptr;
        }
        else {
            digits = nullptr;
        }

        if (digits)	{
            LED_COM_BRIGHTNESS(cn_1, cds.getBrightness());
            if (cn_1 == 0 && *digits == 0)
                LED_DISPLAY_VALUE(12, 0);
            else
                LED_DISPLAY_VALUE(*digits, cn_1);
        }
        else {
            LED_SET_NULL();
        }

        if (++cn > 6)
            cn = 0;

        return;
    }

HANDLE_COM11_SEGS:
    OCR2 = 255;										/* 최대밝기로 */
    if (state == FACTORYTEST_STATE) {
        digitalWrite(SEGS_ARDUINO[0], HIGH);
        digitalWrite(SEGS_ARDUINO[1], HIGH);
        digitalWrite(SEGS_ARDUINO[2], HIGH);
        for (int i = 0; i < 7; i++)
            digitalWrite(SEGS_ARDUINO[8 + i], HIGH);
    }
    else {
        Rtc::DateTime *c = datetime.current();
        /* AM/PM */
        if (c->hour < 12) {
            digitalWrite(SEGS_ARDUINO[0], HIGH);
            digitalWrite(SEGS_ARDUINO[1], LOW);
        } else {
            digitalWrite(SEGS_ARDUINO[0], LOW);
            digitalWrite(SEGS_ARDUINO[1], HIGH);
        }

        /* GPS */
        digitalWrite(SEGS_ARDUINO[2], time_synced_between_gps_n_rtc ? HIGH : LOW);

        /* Weekdays */
        for (int i = 0; i < 7; i++)
            digitalWrite(SEGS_ARDUINO[8 + i], c->wday == i ? HIGH : LOW);
    }

    ++cn;
}

void loop()
{
    if (state == STARTED_STATE) {
        VariableTimedAction::updateActions();
        state = NORMAL_STATE;
        gps.start();
        Serial.println("state: STARTED ==> NORMAL");
    }
    else if (state == FACTORYTEST_STATE) {
        Rtc::DateTime *c = datetime.current();
        static int loop_count = 10;

        matrix.setBrightness(cds.getBrightness());
        for (int i = 0; i < loop_count; i++) {
            c->hour = i;
            c->min = i;
            for (int j = 0; j < 8; j++) {
                matrix.setNumber(j, i, i);
            }
            matrix.write();
            delay(1000);
        }

        c->hour = 8;
        c->min  = 8;
        for (int j = 0; j < 8; j++) {
            matrix.setNumber(j, 8, 8);
        }
        matrix.write();
        delay(3000);


        state = STARTED_STATE;
        Serial.println("state: FACTORYTEST ==> STARTED");
    }
    else if (state == NORMAL_STATE) {
        VariableTimedAction::updateActions();
    }
    else if (state == TIMESETTING_STATE) {
        VariableTimedAction::updateActions();
    }
}
