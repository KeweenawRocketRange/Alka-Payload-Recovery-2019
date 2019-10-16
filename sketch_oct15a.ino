#define MOTOR_FOWARD 10
#define MOTOR_REVERSE 11

void setup() {
  pinMode(MOTOR_FORWARD, OUTPUT);
  pinMode(MOTOR_REVERSE, OUTPUT);
  
}

void loop() {
  

}

void motorF(void){
  digitalWrite(MOTOR_FORWARD, HIGH);
  
}
