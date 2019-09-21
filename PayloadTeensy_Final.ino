/*
IREC PAYLOAD
Keweenaw Rocket Range
Michigan Technological University

David Hoffman
Erik Van Horn

Version 1.0

Summary:  


*/

#include <RF24_config.h>
#include <RF24.h>
#include <printf.h>
#include <nRF24L01.h>
#include <SD_t3.h>	//SD card for teensy
#include <SD.h>
#include <SPI.h>
#include <Wire.h> //to communicate with the BME & MPU
#include <Adafruit_Sensor.h> //for the BME sensor
#include "Adafruit_BME680.h" // for the BME sensor
#include <Adafruit_GPS.h> //for the GPS

//BME Prep
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME680 bme; // I2C

//MPU Prep
const int MPU_addr = 0x68;  // I2C address of the MPU-6050
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;

//GPS Prep
#define mySerial Serial1
Adafruit_GPS GPS(&mySerial);
#define GPSECHO  false
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

//Geiger counter prep

#define LOG_PERIOD 15000  //15000-60000
#define MAX_PERIOD 60000

unsigned long counts;
unsigned long cpm;
unsigned int multiplier;
unsigned long previousMillis;

double usvh;

//------------------------------------------------
//Initializes I2C data and GPS data arrays

const float start = 1234;
const float stop = 4321;

float data[19];

//------------------------------------------------
//Sets SD card location 

const int chipSelect = BUILTIN_SDCARD;

File myFile;

//-----------------------------------------------
//NRF24l01 setup

RF24 radio (7, 8);

byte addresses[][6] = { "1Node", "2Node" };

//------------------------------------------------
//Defines pins for data recording, transmitting LED, & DIP switch

#define LOG_LED 6
#define TX_LED 27
#define BUZZER 26
#define P1 30
#define P2 29
#define P3 28
#define P4 25
#define P5 24
#define P6 4
#define P7 3
#define P8 2
#define REED 32

bool recordData;

byte dipValue = 0;

//Bool for reed switch


//------------------------------------------------

class MPU6050 {

public:

	void begin() {

		Wire.begin();
		Wire.beginTransmission(MPU_addr);
		Wire.write(PWR_MGMT_1);  // PWR_MGMT_1 register
		Wire.write(0);     // set to zero (wakes up the MPU-6050)
		Wire.endTransmission(true);
		Wire.beginTransmission(MPU_addr);
		Wire.write(CONFIG_ACCEL);	//Accel config register
		Wire.write(ACCEL_16);	//Sets measurment range to +- 16g
		Wire.endTransmission(true);
	}

	bool setAccelRange(int num) {

		int range;

		if (num == 16) {
			range = ACCEL_16;
		}
		else if (num == 8) {
			range = ACCEL_8;
		}
		else if (num == 2) {
			range = ACCEL_2;
		}
		else {
			Serial.println("ERROR: INVALID RANGE");
		}

		Wire.beginTransmission(MPU_addr);
		Wire.write(CONFIG_ACCEL);
		Wire.write(range);
		Wire.endTransmission(true);
		return(true);
	}


private:

	const int MPU_addr = 0x68;
	const int CONFIG_ACCEL = 0x1C;
	const int ACCEL_16 = 0b00011000;
	const int ACCEL_8 = 0b00010000;
	const int ACCEL_2 = 0b00000000;
	const int PWR_MGMT_1 = 0x6B;

};

//------------------------------------------------
//Data packet for transmission

struct DataPacket {

	float zAccel;
	float altitude;
	float temperature;
	float pressure;
	
};

struct DataPacket2 {

	float xGyro;
	float yGyro;
	float zGyro;
	float temp;
	float xAccel;
	float yAccel;
	float humidity;
	float gasResistance;
	
};

//------------------------------------------------

void setup() {

	Serial.begin(9600);
	GPS.begin(9600);
	mySerial.begin(9600);
	GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
	GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);
	GPS.sendCommand(PMTK_API_SET_FIX_CTL_5HZ);
#ifdef __arm__
	usingInterrupt = false;  //NOTE - we don't want to use interrupts on the Due
#else
	useInterrupt(true);
#endif
	delay(1000);
	mySerial.println(PMTK_Q_RELEASE);

	//BME Setup

	bme.begin();
	bme.setTemperatureOversampling(BME680_OS_8X);
	bme.setHumidityOversampling(BME680_OS_2X);
	bme.setPressureOversampling(BME680_OS_4X);
	bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
	bme.setGasHeater(320, 150); // 320*C for 150 ms
	//Data packet
	
	 recordData = true;
	

	//MPU Setup

	MPU6050 mpu;

	mpu.begin();

	mpu.setAccelRange(16);

	//LED,buzzer, & DIP switch

	pinMode(TX_LED, OUTPUT);
	pinMode(LOG_LED, OUTPUT);
	pinMode(BUZZER, OUTPUT);
	pinMode(P1, INPUT_PULLUP);
	pinMode(P2, INPUT_PULLUP);
	pinMode(P3, INPUT_PULLUP);
	pinMode(P4, INPUT_PULLUP);
	pinMode(P5, INPUT_PULLUP);
	pinMode(P6, INPUT_PULLUP);
	pinMode(P7, INPUT_PULLUP);
	pinMode(P8, INPUT_PULLUP);
	pinMode(REED, INPUT_PULLUP);

	//Define start and stop points of data array

	data[0] = start;
	
	//Geiger counter setup

	counts = 0;
	cpm = 0;
	usvh = 0.0;
	multiplier = MAX_PERIOD / LOG_PERIOD;
	Serial.begin(9600);
	attachInterrupt(2, tube_impulse, FALLING);

	//nrf24l01 setup

	//RadioSetup();
	

	radio.begin();
	radio.setPALevel(RF24_PA_MAX);
	radio.setDataRate(RF24_250KBPS);
	radio.setChannel(33);
	radio.openWritingPipe(addresses[0]);
	radio.openReadingPipe(1, addresses[1]);
	radio.stopListening();
	//SD card setup using the Teensy's builtin SD card
	SD.begin(chipSelect);
	pinMode(6, OUTPUT);

	myFile = SD.open("DATA.txt", FILE_WRITE);

	if (myFile) {
		myFile.println("							New Flight						");
		myFile.println("Ax,Ay,Az,Gx,Gy,Gz,Temp,Pres,Humid,VOC,Lat,Lon,Speed,Angle,Alt");
		Serial.println("							New Flight						");
		Serial.println("Ax,Ay,Az,Gx,Gy,Gz,Temp,Pres,Humid,VOC,Lat,Lon,Speed,Angle,Alt");
	}

	myFile.close();

	//Signal Startup
	
}


void RadioSetup() {
 
		radio.setChannel(33);


}



//----------------------------------------------------------------------------------------------------

#ifdef __AVR__
// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
	char c = GPS.read();
	// if you want to debug, this is a good time to do it!
#ifdef UDR0
	if (GPSECHO)
		if (c) UDR0 = c;
#endif
}

void useInterrupt(boolean v) {

	if (v) {

		OCR0A = 0xAF;
		TIMSK0 |= _BV(OCIE0A);
		usingInterrupt = true;
	}
	else {
		// do not call the interrupt function COMPA anymore
		TIMSK0 &= ~_BV(OCIE0A);
		usingInterrupt = false;
	}

}
#endif //#ifdef__AVR__

uint32_t timer = millis();

//----------------------------------------------------------------------------------------------------
//Reads GPS data, stores in data array

void GPSdata() {

	// if a sentence is received, we can check the checksum, parse it...

	if (GPS.newNMEAreceived()) {

		if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false

			return;  // we can fail to parse a sentence in which case we should just wait for another

	}

	if (GPS.fix) {

		//Stores GPS data in array
	}
}

//----------------------------------------------------------------------------------------------------
//Reads data from BME 280 & MPU6050, stores in data array

int altCount = 0;

DataPacket packet;
DataPacket2 packet2;

void I2Cdata() {

	Wire.beginTransmission(MPU_addr);
	Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
	Wire.endTransmission(false);
	Wire.requestFrom(MPU_addr, 14, true);  // request a total of 14 registers
	AcX = Wire.read() << 8 | Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
	AcY = Wire.read() << 8 | Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
	AcZ = Wire.read() << 8 | Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
	Tmp = Wire.read() << 8 | Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
	GyX = Wire.read() << 8 | Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
	GyY = Wire.read() << 8 | Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
	GyZ = Wire.read() << 8 | Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

	bme.performReading();

	//Stores BME & MPU data in array

	packet2.xAccel = AcX / 2048.0;
	data[0] = packet2.xAccel;
	packet2.yAccel = AcY / 2048.0;
	data[1] = packet2.yAccel;
	packet.zAccel = AcY / 2048.0;
	data[2] = packet.zAccel;
	packet2.temp = Tmp / 340.00 + 36.53;
	data[3] = packet2.temp;

	packet2.xGyro = GyX;
	data[4] = packet2.xGyro;
	packet2.yGyro = GyY;
	data[5] = packet2.yGyro;
	packet2.zGyro = GyZ;
	data[6] = packet2.zGyro;
	packet.temperature = bme.temperature;
	data[7] = packet.temperature;
	packet.pressure = bme.pressure / 100;
	data[8] = packet.pressure;
	packet2.humidity = bme.humidity;
	data[9] = packet2.humidity;
	packet2.gasResistance = bme.gas_resistance / 1000.0;
	data[10] = packet2.gasResistance;
	
	packet.altitude = 44330.0 * (1.0 - pow(packet.pressure / 1013.25, 0.1903));
	data[11] = packet.altitude;
 

	Serial.println(packet.altitude);

	radio.write(&packet, sizeof(packet));

	digitalWrite(TX_LED, HIGH);
	delay(1);
	digitalWrite(TX_LED, LOW);

	
	
}

//----------------------------------------------------------------------------------------------------
//Reads data from geiger counter, prints in micro severts per hour

void GeigerCounter() {

	unsigned long currentMillis = millis();
	if (currentMillis - previousMillis > LOG_PERIOD) {
		previousMillis = currentMillis;
		cpm = counts * multiplier;
		usvh = cpm / 151.0;

		Serial.print("CPM: ");
		Serial.print(cpm);
		Serial.print(" uSv/hr: ");
		Serial.print(usvh);
		Serial.print("\n");
		counts = 0;
		data[17] = usvh;
	}
}

//Function called on interrupt (Teensy pin 2)

void tube_impulse() {

	counts++;

}

//----------------------------------------------------------------------------------------------------
//Logs data to built in SD card of the Teensy

void logData() {


	myFile = SD.open("DATA.txt", FILE_WRITE);

	if (myFile) {

		for (int i = 1; i < 16; i++) {

			myFile.print(data[i]);
			myFile.print(',');
		
		}
		myFile.println();
	}

	myFile.close();

}

//----------------------------------------------------------------------------------------------------


void loop()                   
{
	recordData = false;

	if (digitalRead(REED) == false) {

		recordData = true;

		for (int i = 0; i < 5; i++) {

			digitalWrite(BUZZER, HIGH);
			delay(100);
			digitalWrite(BUZZER, LOW);
			delay(100);

		}

	}

	while (recordData) {

		GPSdata();
		I2Cdata();
		GeigerCounter();
		logData();
		digitalWrite(BUZZER, HIGH);
		delay(10);
		digitalWrite(BUZZER, LOW);

	}

	



}
