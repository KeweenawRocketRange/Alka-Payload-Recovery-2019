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
const int blinkPin = 28;
const int inPin1 = 30;
const int inPin2 = 34;
const int inPin3 = 38;
const int inPin4 = 42;
float time1 = 0;
float time2 = 0;
float time3 = 0;
float time4 = 0;
float velocity = 0;
//Length of rocket in meters. We live in a society
const int rocketLength = 20;



void setup() {
  Serial.begin(9600); //edit to maximize refresh rate
  pinMode((inPin1, inPin2, inPin3, inPin4), INPUT);

  pinMode(blinkPin, OUTPUT);

}

void loop() {

  //test LED. REMOVE LATER
  digitalWrite(blinkPin, HIGH);
  delay(1000);
  digitalWrite(blinkPin, LOW);
  delay(1000);

  while (true) {
    if (digitalRead(inPin1) != HIGH) {
      time1 = millis();
    }
    if (digitalRead(inPin3) != HIGH && time1 != 0) {
      time3 = millis();
      velocity = rocketLength / (time3 - time1);
      break;
    }

    //This is if we end up getting two sensors
    //  if (digitalRead(inPin2)!=HIGH){
    //    time2=millis();
    //  }
    //  if (digitalRead(inPin4)!=HIGH && time2!=0){
    //    time4=millis();
    //  }

    if (velocity != 0) {
      digitalWrite(blinkPin, HIGH);
      delay(500);
      digitalWrite(blinkPin, LOW);
      delay(500);
    }
  }
}
