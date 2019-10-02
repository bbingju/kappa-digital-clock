/* #include <Adafruit_Sensor.h> */
#include <Arduino.h>
#include <Adafruit_GPS.h>
#include <time.h>
#include <Rtc.h>
#include "lunar_cal.h"
#include "KappaSegment.h"
#include "Button.h"

// 아두이노 보드 설정은 아래 링크의 "MegaCore"를 사용 --> ATmega64선택
// https://github.com/MCUdude/MegaCore
//

typedef struct
{
    volatile uint8_t* port;
    uint8_t pin;
} LED_COMSEG_T;


typedef enum
{
    FACTORYTEST_STATE,
    STARTED_STATE,
    TIMESYNC_STATE,
    NORMAL_STATE,
    TIMESETTING_STATE,
} State;

const uint8_t COMS_ARDUINO[] =
{
    32,														/* COM1 */
    33,														/* COM2 */
    36														/* COM11 */
};

const uint8_t COMS_DECIMAL_PLACES[] =
{
    0,														// COM1
    1,														// COM2
};

const uint8_t COMS_BRIGHTNESS[3][2] =
{
    // low (0)
    {
        8,												// COM1
        8,												// COM2
    },
    // medium (1)
    {
        60,												// COM1
        60,												// COM2
    },
    // high (2)
    {
        180,											// COM1
        180,											// COM2
    },
};

const uint8_t SEGS_ARDUINO[] =
{
    44,		// SEG1
    43,		// SEG2
    42,		// SEG3
    41,		// SEG4
    40,		// SEG5
    39,		// SEG6
    38,		// SEG7
    37,		// SEG8

    8,		// SEG9
    12,		// SEG10
    13,		// SEG11
    14,		// SEG12
    28,		// SEG13
    29,		// SEG14
    30,		// SEG15
    31		// SEG16
};


const uint8_t NUM_TO_SEG[] =
{
    //76543210
    0b01110111,			// 0
    0b00010010,			// 1
    0b01101011,			// 2
    0b01011011,			// 3
    0b00011110,			// 4
    0b01011101,			// 5
    0b01111101,			// 6
    0b00010111,			// 7
    0b01111111,			// 8
    0b01011111,			// 9
    0b00000000,         // NULL
};

void LED_COM_ON(uint8_t com_number)
{
    digitalWrite(COMS_ARDUINO[com_number], HIGH);
};

void LED_COM_OFF(uint8_t com_number)
{
    digitalWrite(COMS_ARDUINO[com_number], LOW);
}

void LED_COM_ALL_OFF()
{
    for (int i = 0; i < 3/* 11 */; i++)
    {
        LED_COM_OFF(i);
    }
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

    for (int i = 0; i < 7; i++)	{
        digitalWrite(SEGS_ARDUINO[i], (v1 & (1 << i)) ? HIGH : LOW);
        digitalWrite(SEGS_ARDUINO[i + 8], (v2 & (1 << i)) ? HIGH : LOW);
    }
}

void LED_SET_NULL()
{
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

time_t utc;
time_t local_time;

#define GPSSerial    Serial1
Adafruit_GPS gps(&GPSSerial);
String NMEA1;									//Variable for first NMEA sentence
String NMEA2;									//Variable for second NMEA sentence
char c;												//to read characters coming from the GPS
boolean gps_parsed = false;

volatile uint8_t brightness = 0;
KappaSegment matrix;

Rtc rtc(5);
uint8_t lunardays[3] = { 1, 1, 1 };
volatile uint8_t cn = 0;
volatile uint8_t cn_1 = 0, cn_2 = 0;
volatile uint8_t *digits = NULL;
volatile uint8_t year_upper = 20;
volatile uint8_t com11_segs = 0;
Rtc::DateTime current = { 0, 0, 0, 1, 0, 1, 0 };

boolean time_synced_between_gps_n_rtc = false;

uint8_t temperature;

uint32_t timer_for_gps = millis();
uint32_t timer_for_cds = millis(); /* 조도 체크 */
uint32_t timer_for_dht = millis();
uint32_t timer_for_no_resp = millis();
uint32_t timer_for_blink = millis();
boolean blink_flag = false;

State state = STARTED_STATE;

static Button btn_reset(49);
static Button btn_select(50);
static Button btn_up(51);
static Button btn_down(52);

uint8_t setting_time_com_number = 0;

time_t makeTime(int year, int month, int day, int hour, int minute, int second)
{
    struct tm user_stime;

    user_stime.tm_year   = 2000 + year - 1900;   // 주의: 년도는 1900년부터 시작
    user_stime.tm_mon    = month - 1;	// 주의: 월은 0부터 시작
    user_stime.tm_mday   = day;
    user_stime.tm_hour   = hour;
    user_stime.tm_min    = minute;
    user_stime.tm_sec    = second;
    user_stime.tm_isdst  = 0;			// 썸머 타임 사용 안함

    return mktime(&user_stime);		// epoch 초 환산
}

time_t makeTime(Rtc::DateTime datetime)
{
    return makeTime(datetime.year, datetime.mon, datetime.day, datetime.hour, datetime.min, datetime.sec);
}

void syncCurrentDateTime()
{
    current = rtc.dateTime();

    /* 만약, 월이나 날이 0이면 임시로 1로 셋팅함 */
    if (current.mon == 0)
        current.mon = 1;

    if (current.day == 0)
        current.day = 1;
}

boolean readGPS()
{
    int count = 0;
    while (!gps.newNMEAreceived()) {
        if (count > 70)
            return false;

        c = gps.read();
        count++;
    }
    return gps.parse(gps.lastNMEA());
}

boolean readGPSAndSync()
{
    boolean ret = readGPS();

    if (ret) {
        utc = makeTime(gps.year, gps.month, gps.day, gps.hour, gps.minute, gps.seconds);
        local_time = utc + 3600 * 9; /* Asia/Seoul */
        if ((gps.year != 0 && gps.month != 0 && gps.day != 0) &&
            local_time != makeTime(current)) {
            struct tm *new_time = localtime(&local_time);
            Serial.println("Time sync!");

            current.year = new_time->tm_year - 100;
            current.mon  = new_time->tm_mon + 1;
            current.day  = new_time->tm_mday;
            current.wday = new_time->tm_wday;
            current.hour = new_time->tm_hour;
            current.min  = new_time->tm_min;
            current.sec  = new_time->tm_sec;

            rtc.stop();
            rtc.setDateTime(&current);
            time_synced_between_gps_n_rtc = true;
            rtc.start();
        }

        Serial.print("\nGPS Time: ");
        Serial.print(gps.hour, DEC); Serial.print(':');
        Serial.print(gps.minute, DEC); Serial.print(':');
        Serial.print(gps.seconds, DEC); Serial.print('.');
        Serial.println(gps.milliseconds);
        Serial.print("Date: ");
        Serial.print(gps.day, DEC); Serial.print('/');
        Serial.print(gps.month, DEC); Serial.print("/20");
        Serial.println(gps.year, DEC);
        Serial.print("Fix: "); Serial.print((int)gps.fix);
        Serial.print(" quality: "); Serial.println((int)gps.fixquality);
    }

    return ret;
}

void setup()
{
    gps.begin(9600);
    //These lines configure the GPS Module
    gps.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    gps.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
    gps.sendCommand(PGCMD_NOANTENNA);

    matrix.setup();

    rtc.setup();

    // SETUP GPIO
    for (int i = 0; i < 11; i++) {
        pinMode(COMS_ARDUINO[i], OUTPUT);
    }

    for (int i=0; i<16; i++) {
        pinMode(SEGS_ARDUINO[i], OUTPUT);
    }

    pinMode(15, OUTPUT);
    //setPwmFrequency(15, 1024);
    digitalWrite(15, HIGH);		// 일단 밝기 조절은 넘어가고... 그냥 항상 ON 시켜놓음

    pinMode(btn_reset.pin, INPUT_PULLUP);
    pinMode(btn_select.pin, INPUT_PULLUP);
    pinMode(btn_up.pin, INPUT_PULLUP);
    pinMode(btn_down.pin, INPUT_PULLUP);

    // OFF All COMs
    LED_COM_ALL_OFF();

    // initialize timer1
    noInterrupts();           // disable all interrupts

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

    interrupts();             // enable all interrupts

    // 실제 동작에서는 COM1 부터 COM11까지 빠른 속도로
    // (Timer사용, 약 10-20ms마다 전환) 전환하며 보여주면 됨
    //-----------------------------------------------


    //-----------------------------------------------
    delay(1000);

    Serial.begin(115200);
    Serial.println("Setup Done.");

    rtc.start();

    /* up & down 버튼이 눌린 상태로 전원이 인가되면 factory test mode 로 진입함. */
    if (btn_down.isPushing() && btn_up.isPushing()) {
        state = FACTORYTEST_STATE;
        Serial.println("state: FACTORYTEST");
    }
}

ISR(TIMER1_COMPA_vect){         //timer0 interrupt

    if (state == STARTED_STATE || state == TIMESYNC_STATE) {
        cn_1 = cn / 2;
        cn_2 = cn % 2;

        LED_COM_ALL_OFF();
        LED_COM_ON(cn_1);

        if (state == TIMESYNC_STATE) {
            if (cn_1 == 2)
                goto HANDLE_COM11_SEGS;
            else
                LED_SET_NULL();
        }
        else {
            LED_SET_NULL();
        }

        if (++cn > 6)
            cn = 0;

        return;
    }
    else if (state == FACTORYTEST_STATE) {
        cn_1 = cn / 2;
        cn_2 = cn % 2;

        LED_COM_ALL_OFF();
        LED_COM_ON(cn_1);

        if (cn_2 == 0) {
            if (cn_1 == 0)       digits = &current.hour;
            else if (cn_1 == 1)  digits = &current.min;
            else if (cn_1 == 2) goto HANDLE_COM11_SEGS;
            else digits = nullptr;
        }
        else {
            digits = nullptr;
        }

        if (digits)	{
            LED_COM_BRIGHTNESS(cn_1, brightness);
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
        cn_1 = cn / 2;
        cn_2 = cn % 2;

        LED_COM_ALL_OFF();
        LED_COM_ON(cn_1);

        if (cn_2 == 0) {
            if (cn_1 == 0)       digits = &current.hour;
            else if (cn_1 == 1)  digits = &current.min;
            else if (cn_1 == 2) goto HANDLE_COM11_SEGS;
            else digits = nullptr;
        }
        else {
            digits = nullptr;
        }

        if (digits)	{
            LED_COM_BRIGHTNESS(cn_1, brightness);
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
        cn_1 = cn / 2;
        cn_2 = cn % 2;
        LED_COM_ALL_OFF();
        LED_COM_ON(cn_1);

        if (cn_2 == 0) {
            if (cn_1 == 0)       digits = (setting_time_com_number == 0) ? (blink_flag ? &current.hour : nullptr) : &current.hour;
            else if (cn_1 == 1)  digits = (setting_time_com_number == 1) ? (blink_flag ? &current.min : nullptr) : &current.min;
            else if (cn_1 == 2) goto HANDLE_COM11_SEGS;
            else digits = nullptr;
        }
        else {
            digits = nullptr;
        }

        if (digits)	{
            LED_COM_BRIGHTNESS(cn_1, brightness);
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
    if (state == TIMESYNC_STATE) {
        digitalWrite(SEGS_ARDUINO[0], LOW);
        digitalWrite(SEGS_ARDUINO[1], LOW);
        digitalWrite(SEGS_ARDUINO[2], HIGH);
        for (int i = 0; i < 7; i++)
            digitalWrite(SEGS_ARDUINO[8 + i], LOW);
    }
    if (state == FACTORYTEST_STATE) {
        digitalWrite(SEGS_ARDUINO[0], HIGH);
        digitalWrite(SEGS_ARDUINO[1], HIGH);
        digitalWrite(SEGS_ARDUINO[2], HIGH);
        for (int i = 0; i < 7; i++)
            digitalWrite(SEGS_ARDUINO[8 + i], HIGH);
    }
    else {
        /* AM/PM */
        if (current.hour <= 12) {
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
            digitalWrite(SEGS_ARDUINO[8 + i], current.wday == i ? HIGH : LOW);
    }

    ++cn;
}

void printCurrentDateTime()
{
    Serial.print("Current - ");
    Serial.print(current.year, DEC); Serial.print('/');
    Serial.print(current.mon, DEC); Serial.print('/');
    Serial.print(current.day, DEC); Serial.print(" ("); Serial.print(current.wday, DEC); Serial.print(')');

    Serial.print(current.hour, DEC); Serial.print(':');
    Serial.print(current.min, DEC); Serial.print(':');
    Serial.print(current.sec, DEC);

    Serial.print(" (");
    Serial.print(lunardays[0], DEC); Serial.print('-');
    Serial.print(lunardays[1], DEC); Serial.print('-');
    Serial.print(lunardays[2], DEC);
    Serial.println(")");
}

float readTemperature()
{
    static float Rt,Adc,T,LnRt;
    static float Ra = 0.00127225169280943;
    static float Rb = 0.000238650944540013;
    static float Rc = 0.0000000794319275047443;
    Adc = (float)analogRead(A1);
    Rt = (10000.0 * Adc)/(1024.0 - Adc);
    LnRt = log(Rt);
    T = (1.0/(Ra + Rb*LnRt + Rc*pow(LnRt, 3.0)))-273.15;
    /* Serial.print("Temperature: "); */
    /* Serial.print(T); */
    /* Serial.println(" *C"); */
    return T;
}

int readCDS()
{
    int val = analogRead(A0);
    /* Serial.print("CDS: "); */
    /* Serial.println(val); */
    return val;
}

// Restarts program from beginning but does not reset the peripherals and registers
void reset()
{
    matrix.turnOffAll();
    asm volatile ("  jmp 0");
}

void loop()
{
    if (timer_for_dht > millis())
        timer_for_dht = millis();

    if (state == STARTED_STATE) {
        time_synced_between_gps_n_rtc = false;
        syncCurrentDateTime();
        getLunarDate(getTotalDaySolar(current.year, current.mon, current.day), lunardays);
        printCurrentDateTime();
        /* state = TIMESYNC_STATE; */
        state = NORMAL_STATE;
        timer_for_no_resp = millis();
        Serial.println("state: STARTED ==> NORMAL");
    }
    else if (state == FACTORYTEST_STATE) {

        static int loop_count = 10;

        matrix.setBrightness(brightness);
        for (int i = 0; i < loop_count; i++) {
            current.hour = i;
            current.min = i;
            for (int j = 0; j < 8; j++) {
                matrix.setNumber(j, i, i);
            }
            matrix.write();
            delay(1000);
        }

        current.hour = 8;
        current.min  = 8;
        for (int j = 0; j < 8; j++) {
            matrix.setNumber(j, 8, 8);
        }
        matrix.write();
        delay(5000);


        state = STARTED_STATE;
        Serial.println("state: FACTORYTEST ==> STARTED");
    }
    else if (state == TIMESYNC_STATE) {
        while (true) {
            if (btn_reset.isPushed()) {
                reset();
                break;
            }

            /* Handle Select Button */
            if (btn_select.isPushed()) {
                setting_time_com_number = 0;
                current.sec = 0;
                blink_flag = false;
                timer_for_no_resp = millis();
                state = TIMESETTING_STATE;
                Serial.println("state: TIMESYNC ==> TIMESETTING");
                break;
            }

            if (timer_for_gps > millis())
                timer_for_gps = millis();

            if (millis() - timer_for_gps > 1000 *  2) {
                timer_for_gps = millis();
                readGPSAndSync();
            }

            if (time_synced_between_gps_n_rtc ||
                millis() - timer_for_no_resp > 1000 * 30) {
                state = NORMAL_STATE;
                Serial.println("state: TIMESYNC ==> NORMAL");
                break;
            }
        }
    }
    else if (state == NORMAL_STATE) {
        while (true) {
            if (btn_reset.isPushed()) {
                reset();
                break;
            }

            /* Handle Select Button */
            if (btn_select.isPushed()) {
                setting_time_com_number = 0;
                current.sec = 0;
                blink_flag = false;
                timer_for_no_resp = millis();
                state = TIMESETTING_STATE;
                Serial.println("state: NORMAL ==> TIMESETTING");
                break;
            }

            /* 조도 체크 통한 밝기 조절 */
            if (timer_for_cds > millis())
                timer_for_cds = millis();

            if (millis() - timer_for_cds > 500) {
                timer_for_cds = millis();	/* reset the timer */

                static int cds_total = 0, cds_avg = 0;
                static int cds_count = 0;
                int cds = readCDS();

                if (cds_count >= 4) {
                    cds_avg = cds_total / cds_count;

                    /* Serial.print("CDS Average: "); */
                    /* Serial.println(cds_avg); */

                    if (cds_avg < 400)
                        brightness = 2;
                    else if (cds_avg > 400 && cds_avg < 900)
                        brightness = 1;
                    else if (cds_avg > 900)
                        brightness = 0;

                    cds_count = 0;
                    cds_total = 0;
                }
                else {
                    cds_total += cds;
                    cds_count++;
                }
            }

            if (timer_for_dht > millis())
                timer_for_dht = millis();

            if (millis() - timer_for_dht > 2000) {
                timer_for_dht = millis();						/* reset the timer */

                float t = readTemperature();
                if (isnan(t)) {
                    Serial.println("Error reading temperature!");
                }
                else {
                    temperature = (uint8_t)t;
                }
            }

            syncCurrentDateTime();
            getLunarDate(getTotalDaySolar(current.year, current.mon, current.day), lunardays);
            /* printCurrentDateTime(); */

            matrix.setBrightness(brightness);
            matrix.setNumber(0, current.sec);
            matrix.setNumber(1, year_upper);
            matrix.setNumber(2, current.year);
            matrix.setNumber(3, current.mon);
            matrix.setNumber(4, current.day);
            matrix.setNumber(5, temperature);
            matrix.setNumber(6, lunardays[1]);
            matrix.setNumber(7, lunardays[2]);
            matrix.write();

            if (timer_for_gps > millis())
                timer_for_gps = millis();

#if 1
            if (!time_synced_between_gps_n_rtc) {
                if (millis() - timer_for_gps > 1000 *  2) {
                    timer_for_gps = millis();

                    Serial.println("Try to sync from GPS.");
                    readGPSAndSync();
                }
            }
            else {
                if (millis() - timer_for_gps > 1000 * 3600) {
                    timer_for_gps = millis();
                    Serial.println("Try to sync from GPS.");
                    readGPSAndSync();
                }
            }
#endif // 0
        }
    }
    else if (state == TIMESETTING_STATE) {
        while (1) {

            if (millis() - timer_for_no_resp > 1000 * 60) {
                setting_time_com_number = 0;
                rtc.setDateTime(&current);
                time_synced_between_gps_n_rtc = false;
                state = NORMAL_STATE;
                Serial.println("state: TIMESETTING ==> NORMAL ");
                break;
            }

            if (timer_for_blink > millis())
                timer_for_blink = millis();

            if (millis() - timer_for_blink > 200) {
                timer_for_blink = millis();
                blink_flag = !blink_flag;
            }

            if (btn_reset.isPushed()) {
                reset();
                break;
            }

            if (btn_select.isPushed()) {
                if (setting_time_com_number == 6) {
                    setting_time_com_number = 0;
                    /* getLunarDate(getTotalDaySolar(current.year, current.mon, current.day), lunardays); */
                    printCurrentDateTime();
                    rtc.stop();
                    rtc.setDateTime(&current);
                    rtc.start();
                    syncCurrentDateTime();
                    printCurrentDateTime();
                    time_synced_between_gps_n_rtc = false;
                    state = NORMAL_STATE;
                    Serial.println("state: TIMESETTING ==> NORMAL ");
                }
                else {
                    if (setting_time_com_number == 1)
                        setting_time_com_number += 3;
                    else
                        setting_time_com_number++;
                }
                break;
            }

            if (btn_up.isPushed()) {
                if (setting_time_com_number == 0) { /* hour */
                    if (current.hour == 24)
                        current.hour = 1;
                    else
                        current.hour++;
                }
                else if (setting_time_com_number == 1) { /* minute */
                    if (current.min == 59)
                        current.min = 0;
                    else
                        current.min++;
                }
                else if (setting_time_com_number == 4) { /* year */
                    if (current.year >= 50)
                        current.year = 0;
                    else
                        current.year++;
                }
                else if (setting_time_com_number == 5) { /* month */
                    if (current.mon == 12)
                        current.mon = 1;
                    else
                        current.mon++;
                }
                else if (setting_time_com_number == 6) { /* day */
                    if (current.day >= getLastDayOfMonth(current.year, current.mon))
                        current.day = 1;
                    else
                        current.day++;
                }

                if (setting_time_com_number >= 4 &&  setting_time_com_number <= 6) {
                    current.wday = getDayOfWeek(current.year, current.mon, current.day);
                    Serial.println(current.wday);
                    getLunarDate(getTotalDaySolar(current.year, current.mon, current.day), lunardays);
                }
                printCurrentDateTime();
            }

            if (btn_down.isPushed()) {
                if (setting_time_com_number == 0) { /* hour */
                    if (current.hour == 1)
                        current.hour = 24;
                    else
                        current.hour--;
                }
                else if (setting_time_com_number == 1) { /* minute */
                    if (current.min == 0)
                        current.min = 59;
                    else
                        current.min--;
                }
                else if (setting_time_com_number == 4) { /* year */
                    if (current.year == 0)
                        current.year = 50;
                    else
                        current.year--;
                }
                else if (setting_time_com_number == 5) { /* month */
                    if (current.mon == 1 || current.mon == 0)
                        current.mon = 12;
                    else
                        current.mon--;
                }
                else if (setting_time_com_number == 6) { /* day */
                    if (current.day == 1 || current.day == 0)
                        current.day = getLastDayOfMonth(current.year, current.mon == 0 ? 1 : current.mon);
                    else
                        current.day--;
                }
                if (setting_time_com_number >= 4 &&  setting_time_com_number <= 6) {
                    current.wday = getDayOfWeek(current.year, current.mon, current.day);
                    Serial.println(current.wday);
                    getLunarDate(getTotalDaySolar(current.year, current.mon, current.day), lunardays);
                }
                printCurrentDateTime();
            }

            matrix.setBrightness(brightness);
            matrix.setNumber(0, current.sec);

            if (blink_flag) {
                matrix.setNumber(1, year_upper);
                matrix.setNumber(2, current.year);
                matrix.setNumber(3, current.mon);
                matrix.setNumber(4, current.day);
            } else {
                if (setting_time_com_number == 4) {
                    matrix.setTurnOff(1);
                    matrix.setTurnOff(2);
                } else if (setting_time_com_number == 5) {
                    matrix.setTurnOff(3);
                } else if (setting_time_com_number == 6) {
                    matrix.setTurnOff(4);
                }
            }

            matrix.setNumber(5, temperature);
            matrix.setNumber(6, lunardays[1]);
            matrix.setNumber(7, lunardays[2]);
            matrix.write();
        }
    }
}
