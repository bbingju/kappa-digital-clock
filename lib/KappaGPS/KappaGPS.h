#ifndef GPS_H
#define GPS_H

#if (ARDUINO >= 100)
  #include <Arduino.h>
#else
  #include <WProgram.h>
#endif

#include <VariableTimedAction.h>
#include <Adafruit_GPS.h>

#define GPSSerial    Serial1

class KappaGPS: public VariableTimedAction {
public:
    KappaGPS(): _gps(&GPSSerial) {}
    ~KappaGPS() {}

    void init(void (*callback)(void *, bool)) {

        _callback = callback;

        _gps.begin(9600);
        //These lines configure the GPS Module
        //_gps.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
        _gps.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
        // _gps.sendCommand(PMTK_SET_NMEA_OUTPUT_ALLDATA);
        _gps.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
        _gps.sendCommand(PGCMD_NOANTENNA);
        // Request updates on antenna status, comment out to keep quiet
        //_gps.sendCommand(PGCMD_ANTENNA);

        delay(1000);

        // Ask for firmware version
        _gps.sendCommand(PMTK_Q_RELEASE);
    }

    Adafruit_GPS *getGPS() {
        return &_gps;
    }

    void printParsed() {
        float s = _gps.seconds + _gps.milliseconds/1000.f + _gps.secondsSinceTime();
        int m = _gps.minute;
        int h = _gps.hour;
        int d = _gps.day;

        while (s >= 60) { s -= 60; m++; }
        while (m >= 60) { m -= 60; h++; }
        while (h >= 24) { h -= 24; d++; }

        Serial.print("\nDate: ");
        Serial.print(_gps.year + 2000, DEC); Serial.print("-");
        if (_gps.month < 10) Serial.print("0");
        Serial.print(_gps.month, DEC); Serial.print("-");
        if (d < 10) Serial.print("0");
        Serial.println(d, DEC);

        Serial.print("   Time: ");
        Serial.print(h, DEC); Serial.print(':');
        Serial.print(m, DEC); Serial.print(':');
        if (s < 10) Serial.print("0");
        Serial.println(s, 3);
        Serial.print("Fix: "); Serial.print((int)_gps.fix);
        Serial.print(" quality: "); Serial.println((int)_gps.fixquality);
        Serial.print("Times [s] since last fix: "); Serial.print(_gps.secondsSinceFix(),3);
        Serial.print(",  GPS time: "); Serial.print(_gps.secondsSinceTime(),3);
        Serial.print(",  GPS date: "); Serial.println(_gps.secondsSinceDate(),3);
        if (_gps.fix) {
            Serial.print("Location: ");
            Serial.print(_gps.latitude, 4); Serial.print(_gps.lat);
            Serial.print(", ");
            Serial.print(_gps.longitude, 4); Serial.println(_gps.lon);
            Serial.print("Speed (knots): "); Serial.println(_gps.speed);
            Serial.print("Angle: "); Serial.println(_gps.angle);
            Serial.print("Altitude: "); Serial.println(_gps.altitude);
            Serial.print("Satellites: "); Serial.println((int)_gps.satellites);
        }
    }

private:
    Adafruit_GPS _gps;
    boolean gps_parsed = false;

    void (*_callback)(void *arg, bool ret) = 0;

    unsigned long run() {
        char c;

        while (GPSSerial.available()) {
            c = _gps.read();
//            Serial.write(c);
        }

        if (_gps.newNMEAreceived()) {

            bool ret = _gps.parse(_gps.lastNMEA());
            if (_callback)
                _callback(this, ret);

            if (ret)
                printParsed();

            // char *s = _gps.lastNMEA();
            // if (!_gps.parse(s)) {   // this also sets the newNMEAreceived() flag to false
            //     if (_callback)
            //         _callback(this, false);
            //     Serial.print("parse error! ");
            //     Serial.println(s);
            // }
            // else {
            //     if (_callback)
            //         _callback(this, true);
            //     printParsed();
            // }
        }
        return 0;
    }
};

#endif /* GPS_H */
