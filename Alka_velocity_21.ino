/*
   The purpose of this program is to determine the speed of the rocket at launch.
   The program will trip a timer once the first IR recerver loses it's signal.
   The timer will end once the second reciever is tripped.
   The speed will be calculated by dividing the known distance of the recievers by the time.
*/



/*
   This Section defines the pins the arduino will use to read the
   sensors.
   At least 2 will be used, but 4 would be best for redundancy.
*/

#include <SD.h>

File file;

const int blinkPin = 28;

const int inPin1 = 30;
const int inPin2 = 34;
const int inPin3 = 38;
const int inPin4 = 42;

const int rfPin = 24;

const int hdwrSS = 53;
const int sck = 52;

float timeDifference1 = 0;
float timeDifference2 = 0;
//time 11 and time 22 are register slaves
long time11 = 0;
long time22 = 0;

float velocity = 0;
//Length of rocket in meters. We live in a society
const int rocketLength = 20;
boolean trip = false;


void setup() {
  Serial.begin(9600); //edit to maximize refresh rate

  pinMode((inPin1, inPin2, inPin3, inPin4), INPUT);

  pinMode((blinkPin, hdwrSS,rfPin,sck), OUTPUT);

  SD.begin(53);

  //timer5 is 16 bit timer
  //starting with a 1 prescaler because the rocket will not take mre than a second to travel the distance of it's length
  cli(); //stops interupts real quick

  TCCR5A = 0;// set entire TCCR1A register to 0
  TCCR5B = 0;// same for TCCR1B
  TCNT5  = 0;//initialize counter value to 0

  sei(); //resume interupts

  digitalWrite(rfPin,HIGH);

  

}



//the loops runs like so:
//in the interest of cycle dependancy, time11 is found at the beginning becuase it takes like 5 cycles or something before the while gets tripped.
//This way I shave off like 1 cycle becuase the while loop is slightly faster than two if statements.
//The while loop is meant to just be active while the rocket is moving through the gate.
//The duty load inside the while loop is like 4, the trip set and the if check.
//Once the if statement is tripped, the rocket has already left the gate, and time efficiency no longer matters.
void loop() {

  time11 = TCNT5 + 1;
  while (digitalRead(inPin1 != HIGH || trip==true)) {
  trip=true;
  if (inPin1 == HIGH) {
    time22 = TCNT5 + 1;
      if (time11 > time22) {
        time22 = time22 + 65535; //if the time overflew, this will artificially fix the overflow for time22
      }
      timeDifference1 = (time22 - time11) * (16 * 10 ^ -6);
      velocity = rocketLength / timeDifference1;

      file = SD.open("velocity.txt", FILE_WRITE);
      Serial.print("Velocity: ");
      Serial.print(velocity);
      Serial.println(" m/s");

      //debugging, remove for final delivery
      Serial.print("Time 1: ");
      Serial.println(timeDifference1);
      Serial.print("Time 2: ");
      Serial.println(timeDifference2);
      Serial.print("Rocket length: ");
      Serial.print(rocketLength);
      Serial.println(" m");
      file.close();

      trip=false; //so we can exit the while loop
    }
    
  }

}
