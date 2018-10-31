/*****************************
 * ENCODER LCD HX711
 * WEMOS D1R1
 * CONNECT MET WEMOS D1R1, 80MHZ, 4M (3M SPIFFS) ON DEV/CU.WCHUSBSERIAL1410
 * LCD 2004 5V GND A4 A5 I2C 0X27
 * ENCODER 3V GND A4 A5 I2C 0X30
 * HX711: 3V GND D2 D9
 * BNO055: 3V GND A4 A5 I2C 0X29
 * UBLOX NEO 6M GPS 5V GND D0 van  TX>RX(D0)
 * SD CARD CATALEX: 5V GND D10 D11 D12 D13 (CS MISO MOSI SCK)
 * 
 * 
 * http://qqtrading.com.my/gps-module-neo-m8n-beidou-glonass
 * 
 * LAST UPDATE 29072018; 10082018 met hx en neiuwe menu; 11082018 met bno055 calib; 13082018 encoder max min issue;
 * 13082018 tarring added, cleaning; 14082018 zeroing bno055 working; 15082018 zeroing niet nodig want boot wijzigt koers?
 * 24082018 ecompass werkt, niet gebruiken voor heading
 * 30082018 gps integratie
 * 31082018 SD card integratie
 * 23092018 in box gesoldeerd
 * 14102018 gps heading, speed
 * 15102018 menu items
 * 20102018 y angle, start/stop recording, box soldering
 * 27102018 klaar!
 * 
 */
 
 /*
  * 
  * Display
  * 
  * strokerate, speed, total strokeforce
  * stroke angle, corrected inangle outangle
  * strokeratio, 
  * max oarspeed, max force
  * 
  */
 
#include <EEPROM.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "i2cEncoderLib.h"
#include "HX711.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SPI.h>
#include <SD.h>
#include <math.h>
#include <elapsedMillis.h>
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>

#define BLYNK_PRINT Serial

File myFile;

elapsedMillis timeElapsed;
elapsedMillis timeElapsed2;

LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display

i2cEncoderLib encoder(0x30); //Initialize the I2C encoder class with the I2C address 0x30 is the address with all the jumper open

/*
 * BNO055
 */

#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055();

/*
 * HX711
 */


#define DOUT  D9
#define CLK  D2

HX711 scale(DOUT, CLK);

long calibration_factor = -100000; //-7050 worked for my 440lb max scale setup

/*
 * GPS
 */

#define GPSSerial Serial

char nmeaSentence[68];
String latitude;    //Latitude
String longitude;   //Longitude
String lndSpeed;    //Speed
String Heading;     //heading cmg true
String gpsTime;   //UTC time, Kuala Lumpur is 8
String localTime;    //Woudrichem tijd

const int numReadings = 30;
int noAvg = 5;

float readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
float total = 0;                  // the running total
float average = 0;                // the average
float Speed = 0;
float SpeedK = 0;
int bnoHeading = 0;
int gpsHeading = 0;
int Bearing = 0;
int oldBearing = 0;
int readwait = 5000;

double googlelat = 0;
double googlelon = 0;
double oldLat = 0;
double oldLng = 0;

/*
 * Encoder
 */

const int IntPin = D8;
int32_t counter;
uint8_t encoder_status;

/*
 *  Variables
 */

String menuItems[] = {"Start recording", "Rower weight", "Oar length", "Save settings", "Calibrate HX711", "Tarring HX711", "Calibrate BNO055", "Zeroing BNO055", "Sensitivity", "Freq meas.heading", "Speed smoothing", "Sens Calibration"};
int menuitems = 11;
int CalibCounterFactor = 100;
int sensi = -10;
int state = 0;
int rowerweight = 80;
float oarlength = 400;
float corrx = 0; float corry = 0; float corrz = 0;
int currangle = 0; int prevangle = 0; int inangle = 0; int outangle = 0;
int anglediff = 0; float anglediffrad = 0;
int currangley = 0; int maxY = 0;
int initBearing = 0;
float currpower = 0;
float currgyro = 0;
float currgyroms = 0; float prevgyroms = 0;
float curracc = 0; 
float currforce = 0;
float totalforce = 0; float totalJoule = 0; float totalWatt = 0;
float strokeratio = 0;
float curaccel = 0;
int strokerate = 0;
int strokeangle = 0;
int tr = 0; int ts = 0; int tsi = 0;
float gy = 0;

const int numReadingsW = 10;

float readingsW[numReadingsW];      // the readings from the analog input
int readIndexW = 0;              // the index of the current reading
float totalW = 0;                  // the running total
float averageWatt = 0;                // the average

unsigned long currentmillis, elapsedtime, totalmillis, strokemillis, elapsedtimestroke;
unsigned long samplemillispre, samplemillispost, samplemillis;
float accmil = 0;

unsigned long timeout;


/*
 *  SD card logging
 */

 int currsm[80];
 float currf[80];
 int curra[80];
 int curray[80];
 float currcc[80];
 int i = 0;
 int di = 0;
 String dataString = "";
 int record = 0;
 const int LED = D7;

 /*
  *   Blynk
  */

char auth[] = "b76670191aaf494c8e830088fc3d9526";
char ssid[] = "telefoon";
char pass[] = "telefoon";



void setup() {

  
  Serial.begin(9600);
  Serial.println("Basic Encoder Test:");

  GPSSerial.begin(9600);

  
  lcd.init();
  lcd.backlight();
  lcd.home();
  lcd.setCursor(3,1); lcd.print("St Ayles Skiff");
  lcd.setCursor(2,2); lcd.print("Roeikrachtmeter");
  delay(2000);
  
  scale.set_scale();
  scale.tare(); //Reset the scale to 0
  long zero_factor = scale.read_average(); //Get a baseline reading

  EEPROM.begin(512);

  pinMode(IntPin, INPUT);
  pinMode(LED, OUTPUT);

  getVars();

  lcd.clear(); 
  lcd.setCursor(3,0); lcd.print("Encoder start");

  encoder.begin((INTE_ENABLE | LEDE_DISABLE | WRAP_DISABLE | DIRE_RIGHT | IPUP_ENABLE | RMOD_X1 ));
  encoder.writeCounter(0);
  encoder.writeMax(11); //Set maximum threshold
  encoder.writeMin(0); //Set minimum threshold

  if (!bno.begin())
    {
        /* There was a problem detecting the BNO055 ... check your connections */
          lcd.clear(); 
          lcd.setCursor(3,1); lcd.print("no BNO055");
        while (1);
    }

  bno.setMode(Adafruit_BNO055::OPERATION_MODE_NDOF);

    int eeAddress = 100;
    long bnoID;
    bool foundCalib = false;

    EEPROM.get(eeAddress, bnoID);

    adafruit_bno055_offsets_t calibrationData;
    sensor_t sensor;

    bno.getSensor(&sensor);
    if (bnoID != sensor.sensor_id)
    {
        lcd.clear(); 
        lcd.setCursor(3,1); lcd.print("No Cal Data");
        delay(500);
    }
    else
    {
         
        lcd.setCursor(3,1); lcd.print("BNO055 Calib");
        eeAddress += sizeof(long);
        EEPROM.get(eeAddress, calibrationData);

        lcd.setCursor(3,2); lcd.print("Restoring");
        bno.setSensorOffsets(calibrationData);
        
        delay(500);
        lcd.setCursor(3,2); lcd.print("          ");
        lcd.setCursor(3,3); lcd.print("Loaded");
        foundCalib = true;
    }

      // see if the card is present and can be initialized:
    if (!SD.begin(D10)) {
      delay(500);
      lcd.setCursor(3,2); lcd.print("          ");
      lcd.setCursor(3,3); lcd.print("          ");
      lcd.setCursor(3,3); lcd.print("SD card fail");

     return;
     }
     delay(500);
      lcd.setCursor(3,2); lcd.print("          ");
      lcd.setCursor(3,3); lcd.print("          ");
      lcd.setCursor(3,2); lcd.print("SD card init");

    delay(500);

    for (int thisReading = 0; thisReading < numReadings; thisReading++) {
        readings[thisReading] = 0;
    }

    for (int thisReadingW = 0; thisReadingW < numReadingsW; thisReadingW++) {
        readingsW[thisReadingW] = 0;
    }

    for (i = 0; i < 80; i = i + 1) {
      currf[i] = 0; // force
      curra[i] = 0;  //  angle x
      curray[i] = 0;  //  angle y
      currsm[i] = 0;  //  samplemillis
      currcc[i] = 0;  //  acceleration
    }

    Blynk.begin(auth, ssid, pass);

    lcd.setCursor(3,3); lcd.print("Blynk start");

    delay(1000);

    timeout = millis();
  
}


void loop() {

  

  Blynk.run();
  
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  //imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  //imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

  tr = 0;

  if (record == 1) {
    digitalWrite(LED, HIGH);
  }
  else {
    digitalWrite(LED, LOW);
  }
  
  
  while (GetGyro() >= sensi) { //*************   recovery
    tr = tr + 1;




  if (digitalRead(IntPin) == LOW) {

  if (encoder.updateStatus()) {
    
      lcd.clear();
      if (encoder.readStatus(E_PUSH)) {
        encoder.writeCounter(0);
        moveSelect(counter);
        }
      counter = encoder.readCounterByte(); 
      showMenu(counter);  
      
    }
  }


  



  
    if (tr == 1) {
    currentmillis = millis();  

    // write stroke data
    
    if (record == 1) {
        for (di = 0; di < tsi; di = di +1) {
      dataString += localTime; dataString += ",";
      dataString += di; dataString += ",";
      dataString += currsm[di]; dataString += ",";
      dataString += curra[di]; dataString += ",";
      dataString += curray[di]; dataString += ",";
      dataString += currf[di]; dataString += ",";
      dataString += currcc[di];

      File dataFile = SD.open("force.txt", FILE_WRITE);
      if (dataFile) {
        dataFile.println(dataString);
        dataFile.close();
      }
      delay(1);
      dataString = "";
    }

      dataString += localTime; dataString += ","; // tijd
      dataString += average; dataString += ",";  // Speed
      dataString += Bearing; dataString += ",";
      dataString += tsi; dataString += ",";          // aantal samples
      dataString += inangle; dataString += ",";     //catch angle
      dataString += outangle; dataString += ",";    //release angle
      dataString += maxY; dataString += ",";        //diepte
      dataString += strokeangle; dataString += ","; //stroke angle
      dataString += strokerate; dataString += ",";  //stroke rate
      dataString += strokeratio; dataString += ","; //stroke ratio
      dataString += totalforce; dataString += ",";  //Newton  
      dataString += totalJoule; dataString += ",";  //Energy
      dataString += totalWatt; dataString += ",";   //Work
      dataString += googlelat; dataString += ",";   //lattitude
      dataString += googlelon;                     //Longitude
      

      File dataFile = SD.open("stroke.txt", FILE_WRITE);
      if (dataFile) {
        dataFile.println(dataString);
        dataFile.close();
      }
      delay(1);
      dataString = "";
    }

    

    // get gps data


    while(GPSSerial.available()) {   //Serial reading
      char c = GPSSerial.read();  //Serial reading
      switch(c)  {
      case '$':         //data separated by $
        GPSSerial.readBytesUntil('*', nmeaSentence, 67);    //* Store data in nmeaSentence
        latitude = parseGprmcLat(nmeaSentence); //Get Latitude
        longitude = parseGprmcLon(nmeaSentence); //Get Longitude
        lndSpeed = parseGprmcSpeed(nmeaSentence); //Get Speed
        Heading = parseGprmcHeading(nmeaSentence); //Get Heading
        gpsTime = parseGprmcTime(nmeaSentence); //Get GPS time
        
        //Serial.println(nmeaSentence);

        if(latitude > "") {  
          googlelat = conv_coords( latitude.toFloat() );        
        }
        if(longitude > "") {   
          googlelon = conv_coords( longitude.toFloat() );
        }  
        if(lndSpeed > "") {  
          Speed = lndSpeed.toFloat();
          SpeedK = Speed * 1.854;
        
        }
        if(gpsTime > "")  {  
          localTime = getlocalTime(gpsTime);  //KL Time
        } 
      }
      
    if (googlelat != oldLat || googlelon != oldLng) {   //latitude > "" && longitude > "" && gpsTime > "" && 

      break;
    }

    
    } //while gps abvailable 


    //avg speed
          
          total = total - readings[readIndex];
          readings[readIndex] = SpeedK;
          total = total + readings[readIndex];
          readIndex = readIndex + 1;

          if (readIndex >= noAvg) {
              readIndex = 0;
            }
          average = total / noAvg;
   
    if (timeElapsed > readwait) {
      
      Bearing = computeBearing(oldLat, oldLng, googlelat, googlelon);
      
      oldLat = googlelat;
      oldLng = googlelon;
      oldBearing = Bearing;
      timeElapsed = 0;
    }

    }
    
    delay(1);
    
  }  // while gyro
   
  ts = 0; totalforce = 0; maxY = 0;
  

            
  while (GetGyro() < sensi) { //**************   stroke
    
    ts = ts + 1;
    
      samplemillispost = millis(); 
      samplemillis = samplemillispost - samplemillispre;
      samplemillispre = millis();   
       
    if (ts == 1)  {
      strokemillis = millis();
      samplemillis = 25;
    }
    
    currangle = GetAngleX() - 180 - initBearing - Bearing; //get current angle
    currangley = GetAngleY();
    delay(6);
    currpower = (scale.get_units()) * -1; // GetPower() convert to positive for direction of bend 
    delay(6);
    curaccel = GetAcc();
    delay(6);
    currgyro = gy / 1000; //GetGyro() / 1000; == in Rad/s gyro.x() * 1000?
    currgyroms = currgyro * (oarlength/100) * -1;  //is de bladsnelheid in m/s 
  

    accmil = (float)samplemillis / 1000; // samplemillis blijft nagenoeg gelijk

    curracc = (currgyroms - prevgyroms) / accmil;  // == dV/dt
    currforce = curracc * currpower;  // == F = m * a == currentpower (kg) * (dV/dt) (m/s2)  == N

    if (curracc > 0) {
      totalforce += currforce;
    }

    
    currsm[ts] = samplemillis;
    curra[ts] = currangle;
    curray[ts] = currangley;
    currf[ts] = currforce;
    currcc[ts] = curracc;

    if (maxY > currangley) {
      maxY = maxY;
    }
    else {
      maxY = currangley;
    }

    delay(5);

    prevgyroms = currgyroms;
    prevangle = currangle;

    if (ts == 1)  {
      inangle = currangle;
    }  
  }

  if (ts >> 3)  {
    outangle = currangle;
    tsi = ts;
    
  }

  strokeangle = outangle - inangle;

  Serial.print("strokeangle: "); Serial.println(strokeangle);

    //convert angle naar radian
    //rad = deg * 1000 / 57296
    anglediffrad = (float)strokeangle * (1000.0/57296.0);

    Serial.print("angleraddiff: "); Serial.println(anglediffrad);
    
    totalJoule = totalforce * ((float)oarlength / 100.0) * anglediffrad; //torque in Joule,energy/work done == Nm  (s=r*rad)

    Serial.print("totaljoule: "); Serial.println(totalJoule);
  
  totalmillis = millis();
  elapsedtime = totalmillis - currentmillis;
  elapsedtimestroke = totalmillis - strokemillis;
  strokeratio = ((float)elapsedtime/(float)elapsedtimestroke);
  strokerate = (60 / (((float)elapsedtime) / 1000.0));

  Serial.print("strokemillis: "); Serial.println(elapsedtimestroke);

  totalWatt = totalJoule / ((float)elapsedtimestroke / 1000.0);

      totalW = totalW - readingsW[readIndexW];
          readingsW[readIndexW] = totalWatt;
          totalW = totalW + readingsW[readIndexW];
          readIndexW = readIndexW + 1;

          if (readIndexW >= 10) {
              readIndexW = 0;
            }
          averageWatt = totalW / 10;

/*
 *   LDC print
 */
 
  if (ts > 5) {
     lcd.clear();
     lcd.setCursor(0,0); lcd.print(inangle); lcd.setCursor(6,0); lcd.print(outangle); lcd.setCursor(12,0); lcd.print(maxY); lcd.setCursor(17,0); lcd.print(strokeangle);
     lcd.setCursor(0,1); lcd.print(strokerate); lcd.print("/min"); lcd.setCursor(9,1); lcd.print(strokeratio); lcd.print(" ratio");
     lcd.setCursor(0,2); lcd.print((average)); lcd.print(" Kph"); lcd.setCursor(10,2); lcd.print(Bearing); lcd.print(" Deg");
     lcd.setCursor(0,3); lcd.print(totalforce); lcd.print(" N"); lcd.setCursor(10,3); lcd.print(totalWatt); lcd.print(" W");
     Blynk.virtualWrite(V1, totalforce);
     Blynk.virtualWrite(V2, totalWatt);
     Blynk.virtualWrite(V9, averageWatt);
     Blynk.virtualWrite(V3, strokerate);
     //Blynk.virtualWrite(V4, strokeangle);
     Blynk.virtualWrite(V5, average);
     Blynk.virtualWrite(V6, inangle);
     Blynk.virtualWrite(V7, outangle);
     Blynk.virtualWrite(V8, maxY);
  }

   ts = 0; prevgyroms = 0;

  prevangle = currangle;
}



//************* FUNCTIONS **************//

void showMenu(int y) {
  
  lcd.clear();
  int p = 0;
  if (y > menuitems) {
    y = menuitems;
  }

  for (int x=((y/4)*4); x<(((y/4)*4)+4); x++) {
    lcd.setCursor(1,p); lcd.print(menuItems[x]);
    p = p + 1;
  }
  if (y > 3  && y < 8) {
    y = y - 4;
  }
  if (y > 7) {
    y = y - 8;
  }
  if (y > menuitems) {
    y = 0;
  }
  lcd.setCursor(0,y); lcd.print(">");
}

void moveSelect(int z) {
  switch (z) {
    
    case 0:
      menuItem1(); //start recording
      break;
    case 1:
      menuItem2(); // rower weight
      break;
    case 2:
      menuItem3(); // oar length
      break;
    case 3:
      menuItem4(); //empty
      break;
    case 4:
      menuItem5(); // calibrate hx711
      break;
    case 5:
      menuItem6(); // calibrate bno055
      break;
    case 6:
      menuItem7();
      break;
    case 7:
      menuItem8();
      break;
    case 8:       // sensitivity
      menuItem9();
      break;
    case 9:       // heading freq read
      menuItem10();
      break;
    case 10:       // speed smoothing
      menuItem11();
      break;
    case 11:       // speed smoothing
      menuItem12();
      break;
  }
  }



void menuItem1() {  // start recording
  
  encoder.writeMax(99); //Set maximum threshold
  encoder.writeMin(-99); //Set minimum threshold
  int stateM = 1;
  lcd.clear();
  lcd.setCursor(2,0); lcd.print("Start");
  lcd.setCursor(2,1); lcd.print("Recording");
    int sens = 0;
  encoder.writeCounter(0);
        encoder.writeMax(1); //Set maximum threshold
        encoder.writeMin(-1); //Set minimum threshold
  delay(500);
  
  //while (digitalRead(IntPin) == LOW) {
  while (stateM = 1) {
  if (encoder.updateStatus()) { 
    }
      if (encoder.readStatus(E_PUSH)) {
        encoder.writeCounter(0);
        
        encoder.writeMax(11); //Set maximum threshold
        encoder.writeMin(0); //Set minimum threshold
        state = 0;
        return;
        
      }
      counter = encoder.readCounterByte(); //Read only the first byte of the counter register
      sens = counter;
      lcd.setCursor(0,3); lcd.print("Startv | Stop^");
      
      if (sens == 1) {
        //start record
        record = 1;
        lcd.setCursor(0,3); lcd.print("Starting?     ");
      }
      if (sens == -1) {
        //stop record
        record = 0;
        lcd.setCursor(0,3); lcd.print("Stopping?      ");
      }

      delay(200);
      
    }
    stateM = 0; 
}

void menuItem2() {  // rower weight
  //  encoder.writeCounter(0);
  encoder.writeMax(99); //Set maximum threshold
  encoder.writeMin(-99); //Set minimum threshold
  int state = 1;
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("Set Rower weight");
  int rw = 0;
  encoder.writeCounter(0);
  delay(500);
  
  //while (digitalRead(IntPin) == LOW) {
  while (state = 1) {
  if (encoder.updateStatus()) { 
    }
      if (encoder.readStatus(E_PUSH)) {
        encoder.writeCounter(0);
        rowerweight = rw;
        state = 0;
        encoder.writeMax(11); //Set maximum threshold
        encoder.writeMin(0); //Set minimum threshold
        return;
        
      }
      counter = encoder.readCounterByte(); //Read only the first byte of the counter register
      rw = rowerweight + counter;
      lcd.setCursor(0,2); lcd.print("weight: ");lcd.print(rw); lcd.print(" Kg ");

    } 
    //state = 0;
    }

void menuItem3() {  // oar length
  //  encoder.writeCounter(0);
  encoder.writeMax(99); //Set maximum threshold
  encoder.writeMin(-99); //Set minimum threshold
  int stateM = 1;
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("Set Oar length");
  lcd.setCursor(2,1); lcd.print("top-cntr/blad");
    int ol = 0;
  encoder.writeCounter(0);
  delay(500);
  
  //while (digitalRead(IntPin) == LOW) {
  while (stateM = 1) {
  if (encoder.updateStatus()) { 
    }
      if (encoder.readStatus(E_PUSH)) {
        encoder.writeCounter(0);
        oarlength = ol;
        state = 0;
        encoder.writeMax(11); //Set maximum threshold
        encoder.writeMin(0); //Set minimum threshold
        return;
        stateM = 0;
      }
      counter = encoder.readCounterByte(); //Read only the first byte of the counter register
      ol =  oarlength + counter;
      lcd.setCursor(0,2); lcd.print("length: ");lcd.print(ol); lcd.print(" cm");
      
    }
    stateM = 0; 
    }

void menuItem4() {  // empty
  lcd.setCursor(3,1);lcd.print("Saving settings");

  EEPROM.write(11, rowerweight);
  EEPROM.commit();
  EEPROMWritelong(21, oarlength);
  EEPROM.commit();
  EEPROMWritelong(31, calibration_factor);
  EEPROM.commit();
  delay(1000);
  
}

void menuItem5() {  // calibrate hx711
  //  encoder.writeCounter(0);
  encoder.writeMax(99); //Set maximum threshold
  encoder.writeMin(-99); //Set minimum threshold
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("Calibrate HX711");
  //long currentMillis;
  long previousMillis = 0;
  long interval = 500;
  //currentMillis = millis();
  int stateM = 1;
  int calibfact = calibration_factor;
  encoder.writeCounter(0);
  delay(500);
  
  while (stateM = 1) {
    
   if (millis() - previousMillis > interval) {
        previousMillis = millis();
        //scale.set_scale(calibration_factor);
        lcd.setCursor(0,3); lcd.print("                  ");
        float wei = scale.get_units();
       lcd.setCursor(0,3); lcd.print(wei);
    }
    
  if (encoder.updateStatus()) { 
    }
      if (encoder.readStatus(E_PUSH)) {
        encoder.writeCounter(0);
        calibration_factor = calibfact;
        state = 0;
        encoder.writeMax(11); //Set maximum threshold
        encoder.writeMin(0); //Set minimum threshold
        return;
        
      }
      counter = encoder.readCounterByte(); //Read only the first byte of the counter register
      calibfact = calibration_factor + (counter*CalibCounterFactor);
 
      lcd.setCursor(0,2); lcd.print("Calibr.F. ");lcd.print(calibfact);
      scale.set_scale(calibfact);
      
    }
    stateM = 0;
}

void menuItem6() {  // tarring hx711

        lcd.clear();
        lcd.setCursor(0,0); lcd.print("Setting tare HX711");
        float weii = scale.get_units();
        lcd.setCursor(13,3); lcd.print(weii);
        delay(1000);
        lcd.setCursor(0,3); lcd.print("  Zeroing  ");
        scale.tare();
        lcd.setCursor(13,3); lcd.print("     ");
        weii = scale.get_units();
        lcd.setCursor(13,3); lcd.print(weii);
        delay(2000);
        state = 0;
        return;

}
      
void menuItem7() {  // calibrate bno055
//********************************
        lcd.clear();
        lcd.setCursor(0,0); lcd.print("Calibrating BNO055");
        delay(500);
        
while (!bno.isFullyCalibrated())
        {
          lcd.setCursor(5,2); lcd.print("Calib status");
            //bno.getEvent(&event);
            /* Optional: Display calibration status */
            displayCalStatus();
            /* Wait the specified delay before requesting new data */
            delay(BNO055_SAMPLERATE_DELAY_MS);
        }
    

    Serial.println("\nFully calibrated!");
    lcd.setCursor(0,3); lcd.print("                    ");
    lcd.setCursor(0,3); lcd.print("Fully calibrated!");
    adafruit_bno055_offsets_t newCalib;
    bno.getSensorOffsets(newCalib);
    delay(500);
    lcd.setCursor(0,3); lcd.print("Storing calib data ");

    int eeAddress = 100;
    long bnoID;
    sensor_t sensor;
    bno.getSensor(&sensor);
    bnoID = sensor.sensor_id;

    EEPROM.put(eeAddress, bnoID);
    EEPROM.commit();

    eeAddress += sizeof(long);
    EEPROM.put(eeAddress, newCalib);
    EEPROM.commit();
    lcd.setCursor(0,3); lcd.print("Data stored @EEPROM");

    delay(1000);
    state = 0;
  return;
}

void menuItem8() { //zeroing heading

        lcd.clear();
        lcd.setCursor(0,0); lcd.print("Zeroing Heading");
        
        lcd.setCursor(13,3); lcd.print(Bearing);
        delay(500);
        lcd.setCursor(0,3); lcd.print("  Zeroing  ");
        
        lcd.setCursor(13,3); lcd.print("     ");
        initBearing = Bearing;
        
        lcd.setCursor(13,3); lcd.print("done!");
        delay(1000);
        state = 0;
        return;

}

        
        
   /*     //magnetic sketch voor magneticheading via bno055
        imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
        imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
        imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

    float nPitch = (atan(accel.x()/sqrt(accel.y()*accel.y() + accel.z()*accel.z())));//*180/PI;
    float nRoll = (atan(accel.y()/sqrt(accel.x()*accel.x() + accel.z()*accel.z())));
   
    float nxh=mag.x()*cos(-1*nPitch)+mag.z()*sin(-1*nPitch);
    float nyh=mag.x()*sin(nRoll)*sin(-1*nPitch)+mag.y()*cos(nRoll)-mag.z()*sin(nRoll)*cos(-1*nPitch);

    float nheading=atan2(nyh, nxh)*180/PI;
  
    if (nheading < 0) {
      nheading = 360 + nheading;
    }
      bnoHeading = nheading;

        corrx =  euler.x();
        corry = euler.y();
        corrz = euler.z();

        return;
  
  delay(500);
}
*/
void menuItem9() {  // sensitivity
  //  encoder.writeCounter(0);
  encoder.writeMax(99); //Set maximum threshold
  encoder.writeMin(-99); //Set minimum threshold
  int stateM = 1;
  lcd.clear();
  lcd.setCursor(2,0); lcd.print("Stroke");
  lcd.setCursor(2,1); lcd.print("Sensitivity");
    int sens = -10;
  encoder.writeCounter(0);
  delay(500);
  
  //while (digitalRead(IntPin) == LOW) {
  while (stateM = 1) {
  if (encoder.updateStatus()) { 
    }
      if (encoder.readStatus(E_PUSH)) {
        encoder.writeCounter(0);
        sensi = sens;
        state = 0;
        encoder.writeMax(11); //Set maximum threshold
        encoder.writeMin(0); //Set minimum threshold
        return;
        
      }
      counter = encoder.readCounterByte(); //Read only the first byte of the counter register
      sens =  sensi + counter;
      lcd.setCursor(0,3); lcd.print("                   ");
      lcd.setCursor(0,3); lcd.print("Sensitivity: ");lcd.print(sens);
      delay(200);
      
    }
    stateM = 0; 
    }

void menuItem10() {   //heading freq reading
    encoder.writeMax(99); //Set maximum threshold
  encoder.writeMin(-99); //Set minimum threshold
  int stateM = 1;
  lcd.clear();
  lcd.setCursor(2,0); lcd.print("Heading");
  lcd.setCursor(2,1); lcd.print("Freq read");
    int sens = readwait;
  encoder.writeCounter(0);
  delay(500);
  
  //while (digitalRead(IntPin) == LOW) {
  while (stateM = 1) {
  if (encoder.updateStatus()) { 
    }
      if (encoder.readStatus(E_PUSH)) {
        encoder.writeCounter(0);
        readwait = sens;
        state = 0;
        encoder.writeMax(11); //Set maximum threshold
        encoder.writeMin(0); //Set minimum threshold
        return;
        
      }
      counter = encoder.readCounterByte(); //Read only the first byte of the counter register
      sens =  readwait + (counter * 100);
      lcd.setCursor(0,3); lcd.print("                   ");
      lcd.setCursor(0,3); lcd.print("ms: ");lcd.print(sens);
      delay(100);
      
    }
    stateM = 0; 
    }

void menuItem11() {   //speed smoothing
    encoder.writeMax(99); //Set maximum threshold
  encoder.writeMin(-99); //Set minimum threshold
  int stateM = 1;
  lcd.clear();
  lcd.setCursor(2,0); lcd.print("Speed");
  lcd.setCursor(2,1); lcd.print("Smoothing");
    int sens = noAvg;
  encoder.writeCounter(0);
  delay(500);
  
  //while (digitalRead(IntPin) == LOW) {
  while (stateM = 1) {
  if (encoder.updateStatus()) { 
    }
      if (encoder.readStatus(E_PUSH)) {
        encoder.writeCounter(0);
        noAvg = sens;
        state = 0;
        encoder.writeMax(11); //Set maximum threshold
        encoder.writeMin(0); //Set minimum threshold
        return;
        
      }
      counter = encoder.readCounterByte(); //Read only the first byte of the counter register
      sens =  noAvg + counter;
      lcd.setCursor(0,3); lcd.print("                   ");
      lcd.setCursor(0,3); lcd.print("No averaging ");lcd.print(sens);
      delay(100);
      
    }
    stateM = 0; 
    }

void menuItem12() {   //calibration
    encoder.writeMax(99); //Set maximum threshold
  encoder.writeMin(-99); //Set minimum threshold
  int stateM = 1;
  lcd.clear();
  lcd.setCursor(2,0); lcd.print("Calibration");
  lcd.setCursor(2,1); lcd.print("Sensitivity");
    int sens = noAvg;
  encoder.writeCounter(0);
  delay(500);
  
  //while (digitalRead(IntPin) == LOW) {
  while (stateM = 1) {
  if (encoder.updateStatus()) { 
    }
      if (encoder.readStatus(E_PUSH)) {
        encoder.writeCounter(0);
        CalibCounterFactor = sens;
        state = 0;
        encoder.writeMax(11); //Set maximum threshold
        encoder.writeMin(0); //Set minimum threshold
        return;
        
      }
      counter = encoder.readCounterByte(); //Read only the first byte of the counter register
      sens =  CalibCounterFactor * pow(10, counter);
      lcd.setCursor(0,3); lcd.print("                   ");
      lcd.setCursor(0,3); lcd.print("CalibrationF: ");lcd.print(sens);
      delay(100);
      
    }
    stateM = 0; 
    }

void getVars() 
  {
  rowerweight = EEPROM.read(11);
  Serial.println(rowerweight);
  oarlength = EEPROMReadlong(21);
  Serial.println(oarlength);
  calibration_factor = EEPROMReadlong(31);
  Serial.println(calibration_factor);
}

void EEPROMWritelong(int address, long value)
      {
      //Decomposition from a long to 4 bytes by using bitshift.
      //One = Most significant -> Four = Least significant byte
      byte four = (value & 0xFF);
      byte three = ((value >> 8) & 0xFF);
      byte two = ((value >> 16) & 0xFF);
      byte one = ((value >> 24) & 0xFF);

      //Write the 4 bytes into the eeprom memory.
      EEPROM.write(address, four);
      EEPROM.write(address + 1, three);
      EEPROM.write(address + 2, two);
      EEPROM.write(address + 3, one);
      EEPROM.commit();
      }

 long EEPROMReadlong(long address)
      {
      //Read the 4 bytes from the eeprom memory.
      long four = EEPROM.read(address);
      long three = EEPROM.read(address + 1);
      long two = EEPROM.read(address + 2);
      long one = EEPROM.read(address + 3);

      //Return the recomposed long by using bitshift.
      return ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF) + ((two << 16) & 0xFFFFFF) + ((one << 24) & 0xFFFFFFFF);
      }

 void displayCalStatus(void)
{  
    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);

    /* Display the individual values */
    lcd.setCursor(0,3); lcd.print("S: ");lcd.print(system, DEC);lcd.print(" G: ");lcd.print(gyro, DEC);
    lcd.print(" A: ");lcd.print(accel, DEC);lcd.print(" M: ");lcd.print(mag, DEC);
}

float GetGyro() {
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  float cgy = gyro.x() * 1000;
  gy = cgy;
  return gy;
}

float GetAcc() {
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  float cacc = accel.x();
  return cacc;
}

float GetPower() {
    float as = (scale.get_units());
    return as;
}

float GetAngleX() {
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    float xs = euler.x();
    return xs;
}

float GetAngleY() {
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    float ys = euler.z();
    return ys;
}


String getlocalTime(String s)
{
  int hour = s.substring(0,2).toInt();
  int minute = s.substring(2,4).toInt();
  int second = s.substring(4,6).toInt();

  hour += 2;

  if(hour > 24)
    hour -= 24;
  s = String(hour) +":"+String(minute) +":"+ String(second);
  return s;
}

/////////////Return Latitude
String parseGprmcLat(String s)
{
  int pLoc = 0; //paramater location pointer
  int lEndLoc = 0; //lat parameter end location
  int dEndLoc = 0; //direction parameter end location
  String lat;

  if(s.substring(0,4) == "GPRM")
  {
    //Serial.println(s);
    for(int i = 0; i < 5; i++)
    {
      if(i < 3) 
      {
        pLoc = s.indexOf(',', pLoc+1);
      }
      if(i == 3)
      {
        lEndLoc = s.indexOf(',', pLoc+1);
        lat = s.substring(pLoc+1, lEndLoc);
      }
      else
      {
        dEndLoc = s.indexOf(',', lEndLoc+1);
        lat = lat + " " + s.substring(lEndLoc+1, dEndLoc);

      }
    }
    return lat; 
  }

}


////////////Return Longitude
String parseGprmcLon(String s)
{
  int pLoc = 0; //paramater location pointer
  int lEndLoc = 0; //lat parameter end location
  int dEndLoc = 0; //direction parameter end location
  String lon;

  if(s.substring(0,4) == "GPRM")
  {
    //Serial.println(s);
    for(int i = 0; i < 7; i++)
    {
      if(i < 5) 
      {
        pLoc = s.indexOf(',', pLoc+1);
      }
      if(i == 5)
      {
        lEndLoc = s.indexOf(',', pLoc+1);
        lon = s.substring(pLoc+1, lEndLoc);
      }
      else
      {
        dEndLoc = s.indexOf(',', lEndLoc+1);
        lon = lon + " " + s.substring(lEndLoc+1, dEndLoc);
      }
    }
    return lon; 
  }
}


////////////Return Speed
String parseGprmcSpeed(String s)
{
  int pLoc = 0; //paramater location pointer
  int lEndLoc = 0; //lat parameter end location
  int dEndLoc = 0; //direction parameter end location
  String lndSpeed;

  if(s.substring(0,4) == "GPRM")
  {
    //Serial.println(s);
    for(int i = 0; i < 8; i++)
    {
      if(i < 7) 
      {
        pLoc = s.indexOf(',', pLoc+1);
      }
      else
      {
        lEndLoc = s.indexOf(',', pLoc+1);
        lndSpeed = s.substring(pLoc+1, lEndLoc);
      }
    }
    return lndSpeed; 
  }
}

////////////Return Heading
String parseGprmcHeading(String s)
{
  int pLoc = 0; //paramater location pointer
  int lEndLoc = 0; //lat parameter end location
  int dEndLoc = 0; //direction parameter end location
  String Heading;

  if(s.substring(0,4) == "GPRM")
  {
    //Serial.println(s);
    for(int i = 0; i < 9; i++)
    {
      if(i < 8) 
      {
        pLoc = s.indexOf(',', pLoc+1);
      }
      else
      {
        lEndLoc = s.indexOf(',', pLoc+1);
        Heading = s.substring(pLoc+1, lEndLoc);
      }
    }
    return Heading; 
  }
}

//////////////Return GPS time
String parseGprmcTime(String s)
{
  int pLoc = 0; //paramater location pointer
  int lEndLoc = 0; //lat parameter end location
  int dEndLoc = 0; //direction parameter end location
  String gpsTime;

  if(s.substring(0,4) == "GPRM")
  {
    //Serial.println(s);
    for(int i = 0; i < 2; i++)
    {
      if(i < 1) 
      {
        pLoc = s.indexOf(',', pLoc+1);
      }
      else
      {
        lEndLoc = s.indexOf(',', pLoc+1);
        gpsTime = s.substring(pLoc+1, lEndLoc);
      }
    }
    return gpsTime; 
  }
}

// Turn char[] array into String object
String charToString(char *c)
{

  String val = "";

  for(int i = 0; i <= sizeof(c); i++) 
  {
    val = val + c[i];
  }

  return val;
}

float conv_coords(float in_coords)
 {
 //Initialize the location.
 float f = in_coords;
 // Get the first two digits by turning f into an integer, then doing an integer divide by 100;
 // firsttowdigits should be 77 at this point.
 int firsttwodigits = ((int)f)/100; //This assumes that f < 10000.
 float nexttwodigits = f - (float)(firsttwodigits*100);
 float theFinalAnswer = (float)(firsttwodigits + nexttwodigits/60.0);
 return theFinalAnswer;
 }


float computeBearing(float lat1, float lon1, float lat2, float lon2)  {

    float teta1 = radians(lat1);
    float teta2 = radians(lat2);
    float delta1 = radians(lat2-lat1);
    float delta2 = radians(lon2-lon1);

    //==================Heading Formula Calculation================//

    float y = sin(delta2) * cos(teta2);
    float x = cos(teta1)*sin(teta2) - sin(teta1)*cos(teta2)*cos(delta2);
    float brng = atan2(y,x);
    brng = degrees(brng);// radians to degrees
    brng = ( ((int)brng + 360) % 360 ); 

    return brng;
  }

