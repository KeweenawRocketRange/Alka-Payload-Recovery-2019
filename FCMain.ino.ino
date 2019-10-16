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
 
  int sampleRate = 10;  //Interval at which average is calculated
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

  //UI variables
  const int buttonPin = 4;            // Button pin
  const int rLedPin = 6;              // Red LED pin
  const int gLedPin = 5;              // Green LED pin
  const int buzzerPin = 7;            // Buzzer pin
  int buttonState = 0;                // Current state of the button, used to detect button presses
  int lastButtonState = 0;            // Previous state of the button, used to detect button presses
  int buttonCounter = 0;              // Counter for button presses
  boolean inButtonSequence = false;   // In a button sequence means the computer is counting how many buttons are being pressed within the next 3 seconds
  boolean checkButtonPresses = false; // Bool used to exit the loop that looks for button presses
  unsigned long startTime;            // Used for a timer

  //Detect Apogee vars
  float altValues[10];
  float averageAlt[10];
  int altCheck = 0;
  int altThreshold = 10;
  float arraySum = 0;
  int distanceFromApogee = 1; //meters below apogee
  
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
 // Setup pins
   pinMode(buttonPin, INPUT); 
   pinMode(gLedPin, OUTPUT);
   pinMode(rLedPin, OUTPUT);
   pinMode(buzzerPin, OUTPUT);
   digitalWrite(buttonPin, HIGH);
   digitalWrite(gLedPin, LOW);
   digitalWrite(rLedPin, HIGH);

   Serial.begin(9600);

   lastButtonState = digitalRead(buttonPin); // Make the inital state of lastButtonState equal the initial state of buttonState

   // Setup loop
   boolean inSetup = true;
   while(inSetup == true){
      // Loop to strictly look for button presses
      while(checkButtonPresses == false){
        buttonState = digitalRead(buttonPin); 
        if (buttonState != lastButtonState) { // This if statement is what detects a button press
          buttonCounter++;
          delay(500); // Avoid pressing button too many times on accident

          // If detects a button press while not in a button sequence, start the button sequence and start the timer
          if(inButtonSequence == false){
            inButtonSequence = true;
            startTime = millis(); // Start timer
          }
        }  

        // Keep looping through to count how many times the button has been pressed until the 3 second timer runs out
        if(((millis() - startTime) >= 3000) && inButtonSequence == true){
            checkButtonPresses = true;
        }
      }

      // Checking button presses 
      // Checks how many times the button was pressed over the last 3 seconds and execute the corresponding statement
      if (checkButtonPresses == true) {
        if(buttonCounter == 1) {
           buzzer(250, 1);
           Serial.println("1 Button Press");
        }
        else if(buttonCounter == 2) {
           buzzer(250, 2);
           Serial.println("2 Button Presses");
           reverseMotor();
        }
        else if(buttonCounter == 3) {
           buzzer(250, 3);
           Serial.println("3 Button Presses");
           inSetup = false; // Exit setup loop
        }
      }

      // Reset everything and go back to the loop looking strictly for button presses
      delay(500);
      buttonCounter = 0;
      checkButtonPresses = false;
      inButtonSequence = 0;
      lastButtonState = digitalRead(buttonPin);
   }

   digitalWrite(gLedPin, HIGH);
}

// Function to activate the buzzer
// buzzerDuration: How long each buzz will last
// buzzerAmount: How many times it will buzz
void buzzer(int buzzerDuration, int buzzerAmount) {
  for(int i = 0; i < buzzerAmount; i++){
    digitalWrite(buzzerPin, HIGH);
    delay(buzzerDuration);
    digitalWrite(buzzerPin, LOW);
    delay(250);
  }
}

// Function to reverse motor
void reverseMotor(){

}

void loop(){
  retrieveDataBMP();
  calcAvg();
  if(!launched)
    detectLaunch();
  if(launched){
    while(true){
      retrieveDataMPU();
      calcMaxAlt();
    }
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
  
    for(int i = 0; i < sampleRate - 1; i++){

    altValues[i] = altValues[i+1];

  }


  altValues[sampleRate-1] = bmp.readAltitude(101500);
  
  for(int i = 0; i < sampleRate; i++){

    arraySum += altValues[i];
  }

  Serial.println(arraySum/sampleRate);

  for(int i = 0; i < sampleRate-1; i++){

    averageAlt[i] = averageAlt[i+1];

  }
    averageAlt[sampleRate - 1] = (arraySum/sampleRate);

  //Compares running average to previous max altitude
  if((arraySum/sampleRate) > maxAlt){

    maxAlt = (arraySum/sampleRate);
    arraySum = 0;
  }

  if(isFalling()){

      arraySum = 0;
      
      while(true){
    
      flashNbeep(); 

      }
    
  }
    arraySum = 0; 
   
}

//***********************************************************************

bool isFalling(){

  for(int i = 0; i < sampleRate; i++){

    if(!((maxAlt - averageAlt[i]) > distanceFromApogee)){ 

      return(false);
    }
  }

  return(true);
  
}

//***********************************************************************

bool deploy(){

  return(true);
    
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
    buzzer(500, 6);
    for(int i =0; i < 7; i++){
      flashNbeep();
    }
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
