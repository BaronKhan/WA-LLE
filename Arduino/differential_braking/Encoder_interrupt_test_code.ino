
//The below must be #defined rather than declared as int due to interrupt scope issues
#define encoder0PinA  2
#define encoder1PinA  3
#define encoder0PinB  4
#define encoder1PinB  5

volatile int encoder0Pos = 0;
volatile int encoder1Pos = 0;

void setup() {
  pinMode(encoder0PinA, INPUT_PULLUP);
  pinMode(encoder0PinB, INPUT_PULLUP);
  pinMode(encoder1PinA, INPUT_PULLUP);
  pinMode(encoder1PinB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoder0PinA), incrementEncoder0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder1PinA), incrementEncoder1, CHANGE);
  Serial.begin (9600);
  Serial.println("start");
}

void loop() {
  double leftSpeed = checkSpeedLeft();
  double rightSpeed = checkSpeedRight();
  Serial.println ("Left spd:");
  Serial.println (leftSpeed, DEC);
  Serial.println ("Right spd:");
  Serial.println (rightSpeed, DEC);
}

double checkSpeedLeft(){
  double period = 250;

  encoder0Pos = 0;
  EIFR = bit (INTF0);
  interrupts();
  delay(period);                  //Wait while we collect interrupts
  noInterrupts();

  //Get speed from encoder position, REPLACE 1024 WITH ACTUAL 'CLICKS' PER ROTATION
  double angle = (double(encoder0Pos)/400.0) * 360.0;
  return (angle / (period/1000));
}

double checkSpeedRight(){
  double period = 500;

  encoder1Pos = 0;
  EIFR = bit (INTF1);
  interrupts();
  delay(period);                  //Wait while we collect interrupts
  noInterrupts();

  //Get speed from encoder position, REPLACE 1024 WITH ACTUAL 'CLICKS' PER ROTATION
  double angle = (double(encoder1Pos)/400.0) * 360.0;
  return (angle / (period/1000));
}

void incrementEncoder0() {
  if (digitalRead(encoder0PinA) == digitalRead(encoder0PinB)) {
    encoder0Pos++;
  } else {
    encoder0Pos--;
  }
  //Serial.println ("Enc0 pos:");
  //Serial.println (encoder0Pos, DEC);
}

void incrementEncoder1() {
  if (digitalRead(encoder1PinA) == digitalRead(encoder1PinB)) {
    encoder1Pos++;
  } else {
    encoder1Pos--;
  }
  //Serial.println ("Enc1 pos:");
  //Serial.println (encoder1Pos, DEC);
}

