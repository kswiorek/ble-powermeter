#include "HX711.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>
#include <EEPROM.h>

// the .0 is important, so the macro inserts it as float
#define sensMax 1553300.0 // The loadcell reading with weightMax of force
#define sensMin 1378800.0 // The loadcell reading with 0 force
#define weightMax 160.0	// weight for calibration
#define radius 0.175	// radius of the crank
#define sensRadius 0.095// distance of the MPU from the center

#define zeroAngle 180 // the angle, where you want to tare - all angles are according to the CPS service specification

#define freq 0.5 //the update frequency, if your device can cope with it, you can increase it to 1 or 2

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

uint32_t cyclingPowerFeatureCharacteristicDef = 0b00000000000000000000000000001000;  // 000000000000000000000b;
uint8_t cyclingPowerFeatureCharacteristicData[4] = { (uint8_t)(cyclingPowerFeatureCharacteristicDef & 0xff), (uint8_t)(cyclingPowerFeatureCharacteristicDef >> 8), (uint8_t)(cyclingPowerFeatureCharacteristicDef >> 16), (uint8_t)(cyclingPowerFeatureCharacteristicDef >> 24) };
byte sensLoc[1] = { 5 };


long reading;
float weight, weightOld, power, rpm, avr;
double angle;
float reg[160];
bool test;
bool tareTest = true;
int samples, cnt, tareCnt;
int8_t tare;
long avgFin;
class MyServerCallbacks : public BLEServerCallbacks {
	void onConnect(BLEServer* pServer) {
		_BLEClientConnected = true;
	}
	;

	void onDisconnect(BLEServer* pServer) {
		_BLEClientConnected = false;
	}
};

void InitBLE() {
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
	pinMode(32, OUTPUT);
	digitalWrite(32, HIGH);
	while (!mpu.begin()) {
		Serial.println("Failed to find MPU6050 chip");
	}
	scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
	mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
	mpu.setFilterBandwidth(MPU6050_BAND_94_HZ);
	mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  
	Serial.begin(115200);
	Serial.println("Start");
	InitBLE();

	pinMode(5, OUTPUT);	  // tare LED setup
	digitalWrite(5, HIGH);
	EEPROM.begin(1);		//read tare
	tare = EEPROM.read(0); 
}
uint8_t blink;
long timeOld;
uint32_t cumCrankRev;
void loop() {
	// put your main code here, to run repeatedly:

	cnt++;
  
	sensors_event_t a, g, temp;
	mpu.getEvent(&a, &g, &temp);
  
	reading = scale.read();
	weight = (reading - sensMin) * (weightMax) / (sensMax - sensMin);
	angle = fmod(((atan2(a.acceleration.x + (g.gyro.z*g.gyro.z*sensRadius), a.acceleration.y)) * 360 / 6.28) + 90, 360.0);  
	
	// if crank is in position and weight isn't changing increment counter
	// after slightly more than two seconds tare and write to eeprom 
	if((abs(angle - zeroAngle) < 10) && (int(rpm) == 0) && tareTest && (abs(weight - weightOld) < 2)) 
	{       
		tareCnt++;
		digitalWrite(5, (blink & 0x8)); // blink LED if in position
		blink++;
		if (tareCnt >= 200)
		{
			tare = weight;
			EEPROM.write(0, tare);
			EEPROM.commit();
			digitalWrite(5, LOW);
			delay(1000);
			digitalWrite(5, HIGH);
			tareCnt = 0;
			
			tareTest = false;
			
		}
	}
	else if(!(abs(angle - zeroAngle) < 10 && int(rpm) == 0)) 
	{
		weightOld = weight;
		tareCnt = 0;
		//tareTest = 1;
		//Serial.println((int)tareTest);
		digitalWrite(5, HIGH);
	}
	weight -= tare;	// subtract tare
	power = (-g.gyro.z*radius*weight);  

	reg[0] = power;

	samples = constrain(abs(4800 / rpm), 1, 160);
	for (int i = 0; i < samples; i++)
	{
		avr = avr + reg[i];
	}  
  
	avr = avr / samples;
	for (int i = 160 - 1; i >= 0; i--)
	{
		reg[i + 1] = reg[i];
	}
  
	rpm = (g.gyro.z / 6.28) * 60;

	avgFin = avgFin + int(avr);
  
	if (cnt >= (int(80 / freq)))
	{
		if (!_BLEClientConnected) offCnt++;
      		else offCnt = 0;
    
      		if (offCnt > 60*freq) esp_deep_sleep_start();
		
		uint16_t timeOut = map(millis(), 0, 1000, 0, 1024);
		
		cumCrankRev += abs(((millis() - timeOld)*rpm) / 600);
		Serial.println(((millis() - timeOld)*rpm) / 600);
		timeOld = millis();
		avgFin = constrain(avgFin / cnt, 0, 2000);

		powerOut = (uint16_t)avgFin;
		cyclingPowerMeasurementCharacteristicData[2] = (uint8_t)(powerOut & 0xff);
		cyclingPowerMeasurementCharacteristicData[3] = (uint8_t)(powerOut >> 8);
		cyclingPowerMeasurementCharacteristicData[4] = (uint8_t)((cumCrankRev/100) & 0xff);
		cyclingPowerMeasurementCharacteristicData[5] = (uint8_t)((cumCrankRev/100) >> 8);
		cyclingPowerMeasurementCharacteristicData[6] = (uint8_t)(timeOut & 0xff);
		cyclingPowerMeasurementCharacteristicData[7] = (uint8_t)(timeOut >> 8);
    
		cyclingPowerMeasurementCharacteristic.setValue(cyclingPowerMeasurementCharacteristicData, 8);
		cyclingPowerMeasurementCharacteristic.notify();
		cyclingPowerFeatureCharacteristic.setValue(cyclingPowerFeatureCharacteristicData, 4);
		cyclingPowerLocationCharacteristic.setValue(sensLoc, 1);
		
		Serial.println(cumCrankRev/100);

		Serial.print("Kąt: ");
		Serial.print(angle);
		Serial.print("   ");

		Serial.print("Sila: ");
		Serial.print(weight);
		Serial.print("   ");

		Serial.print(" Moc: ");
		Serial.print(power);
		Serial.print("   ");

  
		Serial.print(" S. moc: ");
		Serial.print(avgFin);
		Serial.print("   ");
		
		Serial.print(" Kadencja: ");
		Serial.print(rpm);
		Serial.print("   ");


		Serial.print(" Wartość: ");
		Serial.print(reading);
		Serial.print("   ");
  
  
		Serial.println();
		cnt = 0;
		avgFin = 0;
	}
}
