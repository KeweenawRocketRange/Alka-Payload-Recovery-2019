/*
  Keweenaw Rocket Range
  Beyer Alka Challenge 2019
  Flight Computer Main V1.0
  Authors:
          Erik Van Horn
          Paul Mahowald
          Zoe Knoper
          Chris Norton
          Ethan Visscher
          Andrew Frey
          Erin Dolson
          Sean Smith
  Description:
  Updates:
*/

//******************************************************************************
#include <Adafruit_BMP085.h>
#include <Wire.h>
//******************************************************************************

  const int MPU_addr = 0x68;
  const int CONFIG_ACCEL = 0x1C;
  const int ACCEL_16 = 0b00011000;
  const int ACCEL_8 = 0b00010000;
  const int ACCEL_2 = 0b00000000;
  const int PWR_MGMT_1 = 0x6B;
  const int button = 4;               //Arm button
  const int rLED = 6;                 //Red LED
  const int gLED = 5;                 //Green LED
  const int beep = 7;                 //Buzzer

  int sampleRate = 10;                //Interval at which average is calculated
  float maxAlt;                       //Max altitude measured

  float AcX;
  float last10AcX[10];                // last 10 measurements of AcX
  float avgAcX;                       // average of the last 50 values
  float thresholdG = 2;               // the threshold g force for detecting launch, should be set to 5
  float AcY;                          //Acceleration in Y direction
  float AcZ;                          //Acceleration in the Z direction

  bool launched = false;              //Status of launch

  //UI variables
  int buttonState = 0;                // Current state of the button, used to detect button presses
  int lastButtonState = 0;            // Previous state of the button, used to detect button presses
  int buttonCounter = 0;              // Counter for button presses
  boolean inButtonSequence = false;   // In a button sequence means the computer is counting how many buttons are being pressed within the next 3 seconds
  boolean checkButtonPresses = false; // Bool used to exit the loop that looks for button presses
  unsigned long startTime;            // Used for a timer

  //Detect Apogee vars
  float altValues[10];                //Array of 10 raw altitude values
  float averageAlt[10];               //Array of 10 average altitude values
  float arraySum = 0;                 //Sum of raw altitudes used to calculate average
  int distanceFromApogee = 1;         //Meters below apogee

  //Motor control
  const int MOTOR_FORWARD = 10;       //Set to high to drive motor forward
  const int MOTOR_REVERSE = 11;       //Set to high to drive motor in reverse
  const int motor_time_forward = 5000;//Time motor drives forward
  const int motor_time_reverse = 1000;//Time motor drives in reverse

  Adafruit_BMP085 bmp;                //Instance of bmp Adafruit BMP180 class

//******************************************************************************

void setup() {
  // Setup pins
  pinMode(button, INPUT);
  digitalWrite(button, HIGH);
  pinMode(gLED, OUTPUT);
  pinMode(rLED, OUTPUT);
  pinMode(beep, OUTPUT);
  pinMode(A0, INPUT);
  pinMode(MOTOR_FORWARD, OUTPUT);
  pinMode(MOTOR_REVERSE, OUTPUT);

  digitalWrite(button, HIGH);
  digitalWrite(gLED, LOW);
  digitalWrite(rLED, HIGH);

  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(PWR_MGMT_1);             // PWR_MGMT_1 register
  Wire.write(0);                      // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Wire.beginTransmission(MPU_addr);
  Wire.write(CONFIG_ACCEL);           //Accel config register
  Wire.write(ACCEL_16);               //Sets measurment range to +- 16g
  Wire.endTransmission(true);

   Serial.begin(9600);

 if (!bmp.begin())
 {
    Serial.println("BMP180 sensor not found");
    while (1) {}
 }

//******************************************************************************

   Serial.begin(9600);

   userInterface(); // Call to activate the user interface, looks for button presses
   
}

//******************************************************************************

void loop(){
  retrieveDataBMP();
  retrieveDataMPU();
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

//******************************************************************************

// Function that loops through the user interface, looking for button presses
// Function is terminated when the button is pressed 3 times within 3 seconds
// When function is termninated it will never be called again and the rocket will be armed
void userInterface() {
  lastButtonState = digitalRead(button); // Make the inital state of lastButtonState equal the initial state of buttonState

   // Setup loop
   boolean inSetup = true;
   while(inSetup == true){
      // Loop to strictly look for button presses
      while(checkButtonPresses == false){
        buttonState = digitalRead(button);
        if (buttonState != lastButtonState) { // This if statement is what detects a button press
          buttonCounter++;
          delay(200); // Avoid pressing button too many times on accident

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
      lastButtonState = digitalRead(button);
   }

   digitalWrite(gLED, HIGH);
}

//******************************************************************************
// Function to activate the buzzer
// buzzerDuration: How long each buzz will last
// buzzerAmount: How many times it will buzz
void buzzer(int buzzerDuration, int buzzerAmount) {
  for(int i = 0; i < buzzerAmount; i++){
    digitalWrite(beep, HIGH);
    delay(buzzerDuration);
    digitalWrite(beep, LOW);
    delay(250);
  }
}

//******************************************************************************

// Function to reverse motor
void reverseMotor(){
  digitalWrite(MOTOR_REVERSE, HIGH);
  delay(motor_time_reverse);
  digitalWrite(MOTOR_REVERSE, LOW);
}

//******************************************************************************

void retrieveDataMPU(){

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true);  // request a total of 14 registers

  AcX = Wire.read() << 8 | Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)

  //Serial.println(AcX /2048 );

}

//******************************************************************************

void retrieveDataBMP(){

   Serial.print("Temperature = ");
   Serial.print(bmp.readTemperature());
   Serial.println(" *C");
   Serial.print("Altitude = ");
   Serial.print(bmp.readAltitude(101500));
   Serial.println(" meters");
   Serial.println();

}

//******************************************************************************

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
      digitalWrite(MOTOR_FORWARD, HIGH);
      arraySum = 0;
      delay(motor_time_forward);
      digitalWrite(MOTOR_FORWARD, LOW);
      while(true){
        flashNbeep();
      }
  }
    arraySum = 0;
}

//******************************************************************************

bool isFalling(){
  for(int i = 0; i < sampleRate; i++){
    if(!((maxAlt - averageAlt[i]) > distanceFromApogee)){
      return(false);
    }
  }
  return(true);
}

//******************************************************************************

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

//******************************************************************************

boolean detectLaunch (){
  if (avgAcX > thresholdG || -avgAcX > thresholdG){
    Serial.println("LAUNCH DETECTED!");
    buzzer(500, 1);
    /*for(int i =0; i < 7; i++){
      flashNbeep();
    }*/
    launched = true;
    return true;
  }
  Serial.println("Not launched");
  return false;
}

//******************************************************************************

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

//******************************************************************************
