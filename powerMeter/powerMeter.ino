#include <HX711.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>
#include <EEPROM.h>

// the .0 is important, so the macro inserts it as float
#define sensMax -426468.0 // The loadcell reading with weightMax of force (12.84 kg in my case, see below)
#define sensMin 226021.0 // The loadcell reading with 0 force
#define weightMax 128.4  // weight for calibration
#define radius 0.165  // radius of the crank
#define sensRadius 0.112// distance of the MPU from the center

//#define zeroAngle 180 // the angle, where you want to tare - all angles are according to the CPS service specification

#define freq 0.5 //the update frequency, if your device can cope with it, you can increase it to 1 or 2
//#define freq 1    // actualy here send frequency, 0.5 means send every 2 seconds, 1 means every second


// For debug
#define DEBUG true

#define LOADCELL_DOUT_PIN 16
#define LOADCELL_SCK_PIN 17

Adafruit_MPU6050 mpu;
HX711 scale;

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

#define cyclingPowerService BLEUUID((uint16_t)0x1818) //uuid for the CPS from the BLE GATT website

// required characteristcs according to the CPS specification
BLECharacteristic cyclingPowerLocationCharacteristic(BLEUUID((uint16_t)0x2A5D), BLECharacteristic::PROPERTY_READ); 
BLECharacteristic cyclingPowerFeatureCharacteristic(BLEUUID((uint16_t)0x2A65), BLECharacteristic::PROPERTY_READ);
BLECharacteristic cyclingPowerMeasurementCharacteristic(BLEUUID((uint16_t)0x2A63), BLECharacteristic::PROPERTY_NOTIFY);

uint16_t powerOut = 100;  // W, decimal

// all the required definitions in binary
uint16_t cyclingPowerMeasurementCharacteristicDef = 0b0000000000100000; // cycle power config flags
bool _BLEClientConnected = false;
uint8_t cyclingPowerMeasurementCharacteristicData[8] = {(uint8_t)(cyclingPowerMeasurementCharacteristicDef & 0xff), (uint8_t)(cyclingPowerMeasurementCharacteristicDef >> 8),    // flags 
                                                        (uint8_t)(powerOut & 0xff), (uint8_t)(powerOut >> 8),  // inst. power 
                                                         0, 0,  // cum. crank
                                                         0, 0};  // crank time

uint32_t cyclingPowerFeatureCharacteristicDef = 0b00000000000100000000000000001000;  // 000000000000000000000b;
//                                                          ^^                ^     
//                                                          ||                Crank Revolution Data Supported = true
//                                                          Distributed System Support: 00=Legacy Sensor (all power, we must *2), 01=Not for distributed system (all power, we must *2), 10=Can be used in distributed system (if only one sensor connected - Collector may double the value)

uint8_t cyclingPowerFeatureCharacteristicData[4] = { (uint8_t)(cyclingPowerFeatureCharacteristicDef & 0xff), (uint8_t)(cyclingPowerFeatureCharacteristicDef >> 8), (uint8_t)(cyclingPowerFeatureCharacteristicDef >> 16), (uint8_t)(cyclingPowerFeatureCharacteristicDef >> 24) };
byte sensLoc[1] = { 5 };


long reading;    // RAW data from scale
float weight, weightOld, power, rpm, avr, avgRpm, avgRpmPrev;  // vars not initialized! (rpm, weightOld used without init)
double angle;
float reg[160];
bool tareTest = true;
int samples, cnt, tareCnt, offCnt, rc, pos;  //counters
int16_t tare;
long avgFin, j;
uint8_t blink;
long timeOld, timeNow, timeTmp;
uint16_t cumCrankRevHi, cumCrankRevLow, cumCrankRevInc, cumCrankRev;
uint16_t timeOutInc, timeOut, timeOutDec;
sensors_event_t a, g, temp;

class MyServerCallbacks : public BLEServerCallbacks {  // class for disconnect connect events catching
  void onConnect(BLEServer* pServer) {
    _BLEClientConnected = true;
  }
  ;

  void onDisconnect(BLEServer* pServer) {
    _BLEClientConnected = false;
  }
};

void InitBLE() {    // BLE initialization proc
  BLEDevice::init("FT7");
  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  
  // Create the BLE Service
  BLEService *pPower = pServer->createService(cyclingPowerService);

  pPower->addCharacteristic(&cyclingPowerMeasurementCharacteristic);
  cyclingPowerMeasurementCharacteristic.addDescriptor(new BLE2902());

  pPower->addCharacteristic(&cyclingPowerFeatureCharacteristic);
  pPower->addCharacteristic(&cyclingPowerLocationCharacteristic);
  pServer->getAdvertising()->addServiceUUID(cyclingPowerService);

  pPower->start();
  // Start advertising
  pServer->getAdvertising()->start();
}


void setup() {
  pinMode(32, OUTPUT);       // power source for MPU and HX711
  digitalWrite(32, HIGH);    // turn on sensors
  
  Serial.begin(115200);      // serial init
  Serial.println("Start");
  delay(1000);
  
  while (!mpu.begin()) {     // init mpu
    Serial.println("Failed to find MPU6050 chip");
  }
  
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN); // init hx711
  
  mpu.setGyroRange(MPU6050_RANGE_2000_DEG);        // GyroRange +- 2000 deg/s angular velocity allowed (~5.5 rounds per second)
  mpu.setFilterBandwidth(MPU6050_BAND_94_HZ);      // LowPass filter 94Hz
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);    // AccelerometerRange +- 8G of acceleration allowed
    
  InitBLE();

  pinMode(LED_BUILTIN, OUTPUT);      // tare LED setup
  digitalWrite(LED_BUILTIN, HIGH);
  
  EEPROM.begin(2);    //read tare
  tare = EEPROM.read(0); 
  tare = tare + (EEPROM.read(1)<<8); 

  j = 0;
  cnt = 0;
  cumCrankRev = 0;
  timeOut = 0;
  weightOld = 0;
  rpm = 0;
  avgRpm = 0;
  timeOld = millis();
}

void loop() {
  
  cnt++;
  j++;    // iterations counter  
  rc = j % 160;     // rolling counter
  
  mpu.getEvent(&a, &g, &temp);  // get values from mpu

  reading = scale.read();      // get values from scale hx711
  
  weight = (reading - sensMin) * (weightMax) / (sensMax - sensMin);     // calc curently measured weight according to calibration values
  // !!! Change. We don't need angle at all
  //angle = fmod(((atan2(a.acceleration.x + (g.gyro.z*g.gyro.z*sensRadius), a.acceleration.y)) * 360 / (2*M_PI)) + 90, 360.0);  // ?? calc curent angle
    
  // if crank is in position and weight isn't changing increment counter
  // after slightly more than two seconds tare and write to eeprom 
  // !!! Change. We don't need angle at all. Bottom position - (acc.y == 0 && acc.x > 0) or like this
  if((a.acceleration.x > 0) && (abs(a.acceleration.y < 1)) && (int(rpm) == 0) && tareTest && (abs(weight - weightOld) < 2))  {   // check for tare procedure condition       
  //if((abs(angle - zeroAngle) < 10) && (int(rpm) == 0) && tareTest && (abs(weight - weightOld) < 2))  {   // check for tare procedure condition       
    tareCnt++;
    digitalWrite(LED_BUILTIN, (blink & 0x8)); // blink LED if in position
    blink++;
    if (tareCnt >= 200)  {
      tare = weight;
      EEPROM.write(0, tare);
      EEPROM.write(1, tare>>8);
      EEPROM.commit();
      digitalWrite(LED_BUILTIN, LOW);
      delay(1000);
      digitalWrite(LED_BUILTIN, HIGH);
      tareCnt = 0;
      tareTest = false;        // set flag already tared
    }
  }  
  else if(!((a.acceleration.x > 0) && (abs(a.acceleration.y < 1)) && int(rpm) == 0))   {    // check for tare procedure condition
  //else if(!(abs(angle - zeroAngle) < 10 && int(rpm) == 0))   {    // check for tare procedure condition
    weightOld = weight;
    tareCnt = 0;
    //tareTest = 1;
    //Serial.println((int)tareTest);
    digitalWrite(LED_BUILTIN, HIGH);
  }
    
  weight -= tare;  // subtract tare
  power = (-g.gyro.z*radius*weight);  // calc current power

  reg[rc] = power;
  
  rpm = abs((g.gyro.z / (2*M_PI)) * 60);    // MOVED UP. calc of current rpm from angular velocity. ABS() here once, it's doesn't metter wich direction pedals turning
  avgRpm += constrain(rpm, 0, 200); // avgRpm sum for average calc
  
  samples = 4800 / rpm;          // avoid use functions inside constrain(), devision by zero rpm=0 ?????
  samples = constrain(samples, 1, 160);    // limit samples value to fit to reg[] array size (4800= 80Hz(hx711)*60 (measurements count))

  for (int i = 0; i < samples; i++)  {
    pos = (160 + rc - samples + 1 + i) % 160;  // calc array position based on rolling counter rc and samples count
    avr = avr + reg[pos];            // summarize samples count last added array values
  }
  avr = avr / samples;                // calc power average

  avgFin += round(avr);        // accumulate average power values between transmissions. i think round(avr) better then int(avr)
  
  if (cnt >= (80 / freq))   {      // every 1/freq, 80 is the number of samples (loop repeats) per second. (Was int(80/freq), why int()???? )
    if (!_BLEClientConnected) offCnt++;   // update deep sleep counter
    else offCnt = 0;                      // reset deep sleep counter  
    
    if (offCnt > 60*freq) esp_deep_sleep_start();  // go to deep sleep after 0.5 minute of no BLE connection as i think
            
    avgRpm = avgRpm / cnt;

    timeNow = millis();
    
    cumCrankRevInc = ((timeNow - timeOld)*avgRpm) / 600;  // increment value of cumulative crunk revolutions value x100    
    cumCrankRevHi = trunc((cumCrankRevLow + cumCrankRevInc) / 100);         // count of full revolutions, counting partial rev from last calculation
    //timeOutDec = (((cumCrankRevLow / avgRpmPrev) * 60 * 1000) / 100);       // calc of step back for erlier turned part of crank revolution  // devision by zero!!!!!!!!!!!!!!!!!!!!!!!!!
    (avgRpmPrev < 1) ? (timeOutDec = 0) : (timeOutDec = (((cumCrankRevLow / avgRpmPrev) * 60 * 1000) / 100)); // calc of step back for erlier turned part of crank revolution
    cumCrankRevLow = (cumCrankRevLow + cumCrankRevInc) % 100;               // update of partial rev value
    cumCrankRev += cumCrankRevHi;                                           // inc cummulative revolutions count
    
    if (DEBUG) {
      Serial.print("cumCrunkRev inc:  ");
      Serial.print(cumCrankRevInc);
      Serial.print("   cumCrunkRev Hi:  ");
      Serial.print(cumCrankRevHi);
      Serial.print("   cumCrunkRev Low:  ");
      Serial.print(cumCrankRev);
      Serial.print("   cumCrunkRev:  ");
      Serial.println(cumCrankRev);
    }

    if (avgRpm >= 1) {
      timeTmp = timeOld - timeOutDec + ((cumCrankRevHi / avgRpm) * 60 * 1000);// time of last revolution, based on last calc time minus time of prev partial rev plus time of current revs
      timeOut = map(timeTmp, 0, 1000, 0, 1023) % 65536;                       // stretching time for BLE spec
    }    
    
    timeOld = timeNow;     // update time of last send        
    if (DEBUG) {
      Serial.print("TimeOld:  ");
      Serial.print(timeOld);
      Serial.print("   TimeOut dec:  ");
      Serial.print(timeOutDec);
      Serial.print("   TimeOut:  ");
      Serial.println(timeOut);
    }

    avgFin = avgFin / cnt;                  // avoid use functions inside constrain(), avg calc
    avgFin = 2*avgFin;                      // we have 2 cranks but only one sensor
    avgFin = constrain(avgFin, 0, 2000);    // limit average power value of interval
    powerOut = (uint16_t)avgFin;            // convert cumulative power data for send
    
    // prepare sending data array
    cyclingPowerMeasurementCharacteristicData[2] = (uint8_t)(powerOut & 0xff);
    cyclingPowerMeasurementCharacteristicData[3] = (uint8_t)(powerOut >> 8);
    cyclingPowerMeasurementCharacteristicData[4] = (uint8_t)(cumCrankRev & 0xff);
    cyclingPowerMeasurementCharacteristicData[5] = (uint8_t)(cumCrankRev >> 8);
    cyclingPowerMeasurementCharacteristicData[6] = (uint8_t)(timeOut & 0xff);
    cyclingPowerMeasurementCharacteristicData[7] = (uint8_t)(timeOut >> 8);
    
    cyclingPowerMeasurementCharacteristic.setValue(cyclingPowerMeasurementCharacteristicData, 8);   // prepare sending data array
    cyclingPowerMeasurementCharacteristic.notify();                                                 // send data of curent measurement
    cyclingPowerFeatureCharacteristic.setValue(cyclingPowerFeatureCharacteristicData, 4);           // update data of powermeter features
    cyclingPowerLocationCharacteristic.setValue(sensLoc, 1);                                        // update data powermeter location = left crunck
    
    if (DEBUG) {    // print to serial debug data
      Serial.print("cumCrankRev:  ");
      Serial.println(cumCrankRev);

      //Serial.print("Angle: ");
      //Serial.print(angle);
      //Serial.print("   ");

      Serial.print("Weight: ");
      Serial.print(weight);
      Serial.print("   ");

      Serial.print("Power: ");
      Serial.print(power);
      Serial.print("   ");
  
      Serial.print("avgFin: ");
      Serial.print(avgFin);
      Serial.print("   ");
    
      Serial.print("Cadence: ");
      Serial.print(avgRpm);
      Serial.print("   ");

      Serial.print("Reading: ");
      Serial.print(reading);
      Serial.print("   ");
    
      Serial.print("Tare: ");
      Serial.print(tare);
      Serial.print("   ");
  
      Serial.println();
    }
    
    cnt = 0;      // reset main counter
    avgFin = 0;   // reset average power value after send
    avgRpmPrev = avgRpm; // update of prev avgRpm
    avgRpm = 0;   // reset average rpm after send
  }
}
