const int buttonPin = 4;            // Arm button
const int rLedPin = 6;              // Red LED
const int gLedPin = 5;              // Green LED
const int buzzerPin = 7;            // Buzzer
int buttonState = 0;
int lastButtonState = 0;            // Previous state of the button
int buttonCounter = 0;              // Counter for button presses
boolean inButtonSequence = false;
boolean checkButtonPresses = false;
unsigned long startTime;

void setup() {
   // Setup pins
   pinMode(buttonPin, INPUT); 
   pinMode(gLedPin, OUTPUT);
   pinMode(rLedPin, OUTPUT);
   pinMode(buzzerPin, OUTPUT);
   digitalWrite(buttonPin, HIGH);
   digitalWrite(gLedPin, LOW);
   digitalWrite(rLedPin, LOW);

   Serial.begin(9600);

   lastButtonState = digitalRead(buttonPin); 

   // Setup loop
   boolean inSetup = true;
   while(inSetup == true){
      while(checkButtonPresses == false){
        buttonState = digitalRead(buttonPin); 
        if (buttonState != lastButtonState) {
          buttonCounter++;
          Serial.println(buttonCounter);
          delay(500); // Avoid bouncing

          if(inButtonSequence == false){
            inButtonSequence = true;
            startTime = millis();
          }
        }  

        if(((millis() - startTime) >= 3000) && inButtonSequence == true){
            checkButtonPresses = true;
        }
      }

      if (checkButtonPresses == true) {
        if(buttonCounter == 1) {
           digitalWrite(rLedPin, HIGH);
           digitalWrite(gLedPin, LOW);
           Serial.println("Case 1");
        }
        else if(buttonCounter == 2) {
           digitalWrite(rLedPin, LOW);
           digitalWrite(gLedPin, HIGH);
           Serial.println("Case 2");
        }
        else if(buttonCounter == 3) {
           digitalWrite(rLedPin, HIGH);
           digitalWrite(gLedPin, HIGH);
           Serial.println("Case 3");
        }
      }

      delay(500);
      buttonCounter = 0;
      checkButtonPresses = false;
      inButtonSequence = 0;
      lastButtonState = digitalRead(buttonPin);
      digitalWrite(rLedPin, LOW);
      digitalWrite(gLedPin, LOW);
      
   }
}

void loop() {
 
}
