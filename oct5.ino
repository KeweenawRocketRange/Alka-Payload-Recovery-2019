#include <Adafruit_BMP085.h>
#include <Wire.h> //to communicate with the BME & MPU


  const int MPU_addr = 0x68;
  const int CONFIG_ACCEL = 0x1C;
  const int ACCEL_16 = 0b00011000;
  const int ACCEL_8 = 0b00010000;
  const int ACCEL_2 = 0b00000000;
  const int PWR_MGMT_1 = 0x6B;
  const int button = 4;        //Arm button
  const int rLED = 5;            //Red LED
  const int gLED = 6;            //Green LED
  const int beep = 7;            //Buzzer
 
  int sampleRate = 50;  //Interval at which average is calculated
  float maxAlt;         //Max altitude measured
  int count = 0;

  float AcX;
  float last10AcX[10]; // last 10 measurements of AcX 
  float avgAcX; // average of the last 50 values
  float thresholdG = .4; // the threshold g force for detecting launch, should be set to 5
  float AcY;
  float AcZ;
  float Tmp;
  float GyX;
  float GyY;
  float GyZ;

  bool launched = false;
  
  Adafruit_BMP085 bmp;
  
void setup() {
  pinMode(button, INPUT);
  digitalWrite(button, HIGH);
  pinMode(gLED, OUTPUT);
  pinMode(rLED, OUTPUT);
  pinMode(beep, OUTPUT);
  pinMode(A0, INPUT);
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(PWR_MGMT_1);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Wire.beginTransmission(MPU_addr);
  Wire.write(CONFIG_ACCEL); //Accel config register
  Wire.write(ACCEL_16); //Sets measurment range to +- 16g
  Wire.endTransmission(true);

   Serial.begin(9600);
   
 if (!bmp.begin())
 {
    Serial.println("BMP180 sensor not found");
    while (1) {}
 }
}

void loop(){
  retrieveDataMPU();
  retrieveDataBMP();
  calcAvg(); 
  count++;
  calcMaxAlt();
  if(!launched)
    detectLaunch();
  if(count == sampleRate){
    detectApogee();
    count = 0;
  }
}

void retrieveDataMPU(){

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

  
  Serial.println(AcX /2048 );
  
}

void retrieveDataBMP(){

   Serial.print("Temperature = ");
   Serial.print(bmp.readTemperature());
   Serial.println(" *C");
   Serial.print("Altitude = ");
   Serial.print(bmp.readAltitude(101500));
   Serial.println(" meters");
   Serial.println();

}

boolean detectApogee(){
  if(bmp.readAltitude(101500) < maxAlt){
    sampleRate = 10;
    return(true);
    Serial.println("Apogee Detected");
  }
  return(false);
}

void calcMaxAlt(){
  float currentAlt = bmp.readAltitude(101500);
  if(currentAlt > maxAlt){
    maxAlt = currentAlt;
  }
}

void calcAvg(){
  int size  = 10;
  Serial.print("Size: ");
  Serial.println(size);
  avgAcX = 0.0;
  Serial.print("Last 50 elements: ");
  for(int i = 0; i < size -1; i ++){ // shifts the values down by one
    last10AcX[i]=last10AcX[i +1];
    Serial.print(last10AcX[i]);
    Serial.print("   ");
  }
  Serial.println();
  last10AcX[9] = AcX /2048; // adds most recent value
  for (int i = 0; i < size; i++){ // calculates average
    avgAcX = avgAcX + last10AcX[i];
  }
  avgAcX = avgAcX/10.0;
  Serial.print("Average: ");
  Serial.println(avgAcX);
}
boolean detectLaunch (){  
  if (avgAcX > thresholdG || -avgAcX > thresholdG){
    Serial.println("LAUNCH DETECTED!");
    launched = true;
    return true;
  }
  Serial.println("Not launched");
  return false; 
}

void flashNbeep(){
  digitalWrite(rLED, HIGH);
  digitalWrite(gLED, LOW);
  digitalWrite(beep, HIGH);
  delay(100);
  digitalWrite(gLED, HIGH);
  digitalWrite(rLED, LOW);
  digitalWrite(beep, LOW);
  delay(100);
  //Serial.println(maxAlt);
}
