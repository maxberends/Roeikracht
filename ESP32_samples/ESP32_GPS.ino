/*
 * 
 * 
 * 
 *  WEMOS LOLIN32 OLED 460800
 *  NEO GPS 7M OP 16 EN 5 (TX, RX) 3.3V GND
 * 
 * 
 * 
 * Serial.println(gps.location.lat(), 6); // Latitude in degrees (double)
Serial.println(gps.location.lng(), 6); // Longitude in degrees (double)
Serial.print(gps.location.rawLat().negative ? "-" : "+");
Serial.println(gps.location.rawLat().deg); // Raw latitude in whole degrees
Serial.println(gps.location.rawLat().billionths);// ... and billionths (u16/u32)
Serial.print(gps.location.rawLng().negative ? "-" : "+");
Serial.println(gps.location.rawLng().deg); // Raw longitude in whole degrees
Serial.println(gps.location.rawLng().billionths);// ... and billionths (u16/u32)
Serial.println(gps.date.value()); // Raw date in DDMMYY format (u32)
Serial.println(gps.date.year()); // Year (2000+) (u16)
Serial.println(gps.date.month()); // Month (1-12) (u8)
Serial.println(gps.date.day()); // Day (1-31) (u8)
Serial.println(gps.time.value()); // Raw time in HHMMSSCC format (u32)
Serial.println(gps.time.hour()); // Hour (0-23) (u8)
Serial.println(gps.time.minute()); // Minute (0-59) (u8)
Serial.println(gps.time.second()); // Second (0-59) (u8)
Serial.println(gps.time.centisecond()); // 100ths of a second (0-99) (u8)
Serial.println(gps.speed.value()); // Raw speed in 100ths of a knot (i32)
Serial.println(gps.speed.knots()); // Speed in knots (double)
Serial.println(gps.speed.mph()); // Speed in miles per hour (double)
Serial.println(gps.speed.mps()); // Speed in meters per second (double)
Serial.println(gps.speed.kmph()); // Speed in kilometers per hour (double)
Serial.println(gps.course.value()); // Raw course in 100ths of a degree (i32)
Serial.println(gps.course.deg()); // Course in degrees (double)
Serial.println(gps.altitude.value()); // Raw altitude in centimeters (i32)
Serial.println(gps.altitude.meters()); // Altitude in meters (double)
Serial.println(gps.altitude.miles()); // Altitude in miles (double)
Serial.println(gps.altitude.kilometers()); // Altitude in kilometers (double)
Serial.println(gps.altitude.feet()); // Altitude in feet (double)
Serial.println(gps.satellites.value()); // Number of satellites in use (u32)
Serial.println(gps.hdop.value()); // Horizontal Dim. of Precision (100ths-i32)
 */


#include <TinyGPS++.h>
#include <HardwareSerial.h>



TinyGPSPlus gps;
HardwareSerial SerialGPS(1);

double dist = 0;


#define TASK_SERIAL_RATE 500

uint32_t nextSerialTaskTs = 0;

void setup() {

    Serial.begin(115200);
    SerialGPS.begin(9600, SERIAL_8N1, 5, 16);

}


void loop() {


    while (SerialGPS.available() > 0) {
        gps.encode(SerialGPS.read());
    }


    if (nextSerialTaskTs < millis()) {

        Serial.print("LAT ");  Serial.print(gps.location.lat(), 6);
        Serial.print("  LONG "); Serial.println(gps.location.lng(), 6);
        Serial.print("ALT: ");  Serial.println(gps.altitude.meters());
        Serial.print("Sats: "); Serial.println(gps.satellites.value());
        Serial.print("Speed: "); Serial.println(gps.speed.kmph());
        Serial.print("COG: "); Serial.println(gps.course.deg());
        Serial.print("HDOP: "); Serial.println(gps.hdop.value());
        
        nextSerialTaskTs = millis() + TASK_SERIAL_RATE;
    }



}
