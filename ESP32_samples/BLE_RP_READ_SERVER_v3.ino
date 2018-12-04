/*
 * 
 *  St Ayles Power Oar BLE sensor sketch
 *  
 *  30102018 esp32 with BNO055 & HX711
 *  06112018 v1.0 based on A Spiess's H7 sketch, N Kolbans BLE lib
 *  30112018 added calibration touch menu
 *  
 *  14/11/2018 
 * 
 * 
 * 
 */

 /* ESP32 CONNECTIONS LOLIN32

   BNO055
   ===========
   0x29
   Connect SCL to 22
   Connect SDA to 21
   Connect VDD to 3.3V DC
   Connect GROUND to common ground

   HX711
   ===========
   Connect SCL to 32
   Connect SDA to 33
   Connect VDD to 3.3V DC
   Connect GROUND to common ground   

   TARE/CALIB TOUCH BUTTONS
   ===========
   TARE BNO 2
   TARE TO 15
   CALIBRATION UP 14
   CALIBRATION DOWN 12
   3.3V VIA BUTTON AND LED/REISTANCE TO GROUND

   arduino ide via WEMOS LOLIN32, 80Mhz, 115200



  
   Touch Sensor Pin Layout
   T0 = GPIO4
   T1 = GPIO0
   T2 = GPIO2
   T3 = GPIO15
   T4 = GPIO13
   T5 = GPIO12
   T6 = GPIO14
   T7 = GPIO27
   T8 = GPIO33
   T9 = GPIO32 
*/

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <HX711_ADC.h>



bool _BLEClientConnected = false;

#define rowPowerService BLEUUID((uint16_t)0x1826)

//  power measurement
BLECharacteristic rowPowerMeasurementCharacteristics(BLEUUID((uint16_t)0x2723), 
                BLECharacteristic::PROPERTY_READ | 
                BLECharacteristic::PROPERTY_WRITE);

// angle measurement
BLECharacteristic rowAngleXMeasurementCharacteristics(BLEUUID((uint16_t)0x2763), 
                BLECharacteristic::PROPERTY_READ | 
                BLECharacteristic::PROPERTY_WRITE);

BLECharacteristic sampleTimeMeasurementCharacteristics(BLEUUID((uint16_t)0x2765), 
                BLECharacteristic::PROPERTY_READ | 
                BLECharacteristic::PROPERTY_WRITE);

BLEDescriptor rowPowerDescriptor(BLEUUID((uint16_t)0x2901));
BLEDescriptor rowAngleXDescriptor(BLEUUID((uint16_t)0x2901));
BLEDescriptor sampleTimeDescriptor(BLEUUID((uint16_t)0x2901));


/*
0x2723  force (newton)  org.bluetooth.unit.force.newton
0x2726  power (watt)  org.bluetooth.unit.power.watt
0x2743  angular velocity (radian per second)  org.bluetooth.unit.angular_velocity.radian_per_second
0x2744  angular acceleration (radian per second squared)  org.bluetooth.unit.angular_acceleration.radian_per_second_squared
0x2763  plane angle (degree)  org.bluetooth.unit.plane_angle.degree
0x2765  sample time (ms)  
0x27BC  stroke (per minute) org.bluetooth.unit.stroke_per_minute
0x27BD  pace (kilometre per minute) org.bluetooth.unit.velocity.kilometer_per_minute
*/

#define DOUT 33
#define CLK 32

HX711_ADC LoadCell(DOUT, CLK);  //HX711 scale(DOUT, CLK);



long t;


#define BNO055_SAMPLERATE_DELAY_MS (10)

Adafruit_BNO055 bno = Adafruit_BNO055();

// HX711 POWERMETER VARIABLES

float calibration_factor = -100000; //-7050 worked for my 440lb max scale setup
float as = 0;
int MeasurementsToAverage = 16;
float AveragePower = 0;
float totalp = 0;
int w = 1;
float pas = 20;
unsigned long currentmillis, elapsedtime, totalmillis, strokemillis, elapsedtimestroke, samplemillis;
float pastotal = 0;

float angleX = 0;
float tarreA = 0;

//  Touch

int threshold = 40;
bool buttonONE = false;
bool buttonTWO = false;
bool buttonTHREE = false;
bool buttonFOUR = false;
bool state = false;
const int  ledpin = 27;

void gotButton1(){
 buttonONE = true;
}
void gotButton2(){
 buttonTWO = true;
}
void gotButton3(){
 buttonTHREE = true;
}
void gotButton4(){
 buttonFOUR = true;
}

class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      _BLEClientConnected = true;
    };
    void onDisconnect(BLEServer* pServer) {
      _BLEClientConnected = false;
    }
};

union cvt {
  float valF;
  unsigned char b[4];
  } x;


void InitBLE() {
  BLEDevice::init("READ");
  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pRow = pServer->createService(rowPowerService);

  pRow->addCharacteristic(&rowPowerMeasurementCharacteristics);
  rowPowerDescriptor.setValue("Rate from 0 to 200");
  rowPowerMeasurementCharacteristics.addDescriptor(&rowPowerDescriptor);
  rowPowerMeasurementCharacteristics.addDescriptor(new BLE2902());

  pRow->addCharacteristic(&rowAngleXMeasurementCharacteristics);
  rowAngleXDescriptor.setValue("Rate from 0 to 360");
  rowAngleXMeasurementCharacteristics.addDescriptor(&rowPowerDescriptor);
  rowAngleXMeasurementCharacteristics.addDescriptor(new BLE2902());

  pRow->addCharacteristic(&sampleTimeMeasurementCharacteristics);
  sampleTimeDescriptor.setValue("ms");
  sampleTimeMeasurementCharacteristics.addDescriptor(&rowPowerDescriptor);
  sampleTimeMeasurementCharacteristics.addDescriptor(new BLE2902());
  
  pServer->getAdvertising()->addServiceUUID(rowPowerService);

  pRow->start();
  // Start advertising
  pServer->getAdvertising()->start();
}


void setup() {

  Wire.begin(21, 22, 400000); // (SDA, SCL) (21, 22) are default on ESP32, 400 kHz I2C bus speed
  delay(5000);
  
  Serial.begin(115200);
  Serial.println("Start");
  InitBLE();


  LoadCell.begin();
  long stabilisingtime = 2000; // tare preciscion can be improved by adding a few seconds of stabilising time
  LoadCell.start(stabilisingtime);
  LoadCell.setCalFactor(696.0); // user set calibration factor (float)
  Serial.println("Startup + tare is complete");

  
  // Initialise the sensor BNO055 
  if(!bno.begin())
  {
    // There was a problem detecting the BNO055 ... check your connections 
    Serial.print("BNO055 not detected ");
    while(1);
  }

  delay(1000);

  /* Display the current temperature */
  int8_t temp = bno.getTemp();

  bno.setExtCrystalUse(true);

  touchAttachInterrupt(T2, gotButton1, threshold); // 2
  touchAttachInterrupt(T3, gotButton2, threshold); // 15
  touchAttachInterrupt(T5, gotButton3, threshold); // 12
  touchAttachInterrupt(T6, gotButton4, threshold); // 14
  pinMode(ledpin, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  currentmillis = millis();


  //receive from serial terminal
  if (Serial.available() > 0) {
    float i;
    char inByte = Serial.read();
    if (inByte == 'l') i = -1.0;
    else if (inByte == 'L') i = -10.0;
    else if (inByte == 'h') i = 1.0;
    else if (inByte == 'H') i = 10.0;
    else if (inByte == 't') LoadCell.tareNoDelay();
    else if (inByte == 'b') tarreBNOangle();
    if (i != 't') {
      float v = LoadCell.getCalFactor() + i;
      LoadCell.setCalFactor(v);
    }
  }  
  
  if ((buttonONE == true) && (buttonTWO == true)){
    Serial.println("haha");
    state = true;
    delay(1000);
    buttonONE = false; buttonTWO = false;
  }
  
  if ((buttonONE == true) || (buttonTWO == true)){
    Serial.println("een druk 2");
    buttonONE = false; buttonTWO = false;
  }

  while (state == true){
    digitalWrite(ledpin, HIGH);
    if (buttonONE){
      delay(10);
      LoadCell.tareNoDelay();
      delay(10);
      buttonONE = false;
      Serial.println("button 1 detected");
      state = false;
      digitalWrite(ledpin, LOW);
    }
    if (buttonTWO){
      tarreBNOangle();
      buttonTWO = false;
      Serial.println("button 2 detected");
      state = false;
      digitalWrite(ledpin, LOW);
    }
    if (buttonTHREE){
      float i = 1;
      float v = LoadCell.getCalFactor() + i;
      LoadCell.setCalFactor(v);
      Serial.println(v);
      buttonTHREE = false;
    }
    if (buttonFOUR){
      float i = -1;
      float v = LoadCell.getCalFactor() + i;
      LoadCell.setCalFactor(v);
      Serial.println(v);
      buttonFOUR = false;
    }
    pas = GetPower();
    char txStringP[5];
    dtostrf(pas, 5, 2, txStringP);

    rowPowerMeasurementCharacteristics.setValue(txStringP);
  }




  pas = GetPower();
  angleX = GetAngleX();
  Serial.print(tarreA); Serial.println(angleX);


  delay(BNO055_SAMPLERATE_DELAY_MS);

  char txStringP[5];
  dtostrf(pas, 5, 2, txStringP);

    rowPowerMeasurementCharacteristics.setValue(txStringP);

  char txStringA[5];
  dtostrf(angleX, 5, 2, txStringA);

    rowAngleXMeasurementCharacteristics.setValue(txStringA);

  samplemillis = millis() - currentmillis;
  
  char txStringT[5];
  dtostrf(samplemillis, 5, 2, txStringT);

    sampleTimeMeasurementCharacteristics.setValue(txStringT);
  
  delay(10);
}



float GetPower() 
    {
      LoadCell.update();
      float as = LoadCell.getData() * 100;
      return as;
    }

float GetAngleX()
    {
        imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
        float ang = euler.x() - tarreA;
        if (ang < 0) {
          ang = ang + 359;
        }
        return ang;

  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
    }

void tarreBNOangle()
    {
      imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
      float tarreang = euler.x();
      tarreA = tarreang;
      return;
    }



