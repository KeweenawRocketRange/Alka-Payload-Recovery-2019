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

void setup() {
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

void loop() {
 
}
