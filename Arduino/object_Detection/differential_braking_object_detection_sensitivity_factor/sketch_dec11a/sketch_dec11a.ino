//**************************************  PIN DEFINITIONS  ********************************
//The below must be #defined rather than declared as int due to interrupt scope issues
#define encoder0PinA  2
#define encoder1PinA  3
#define encoder0PinB  4
#define encoder1PinB  5

int inA1 = 6;
int inA2 = 7;
int inB1 = 8;
int inB2 = 9;
int inC1 = 10;
int inC2 = 11;
int inD1 = 12;
int inD2 = 13;

int LED0 = 31;
int LED1 = 33;
int LED2 = 35;
int LED3 = 37;
int LED4 = 39;
int LED5 = 41;
int LED6 = 43;
int LED7 = 45;
int LED8 = 47;
int LED9 = 49;
int LED10 = 51;
int LED11 = 53;

//**************************************  GLOBAL VARIABLES  ********************************
volatile int encoder0Pos = 0;
volatile int encoder1Pos = 0;
double m1pos = 0;
double m2pos = 0;

//**********************************  INTERRUPT SERVICE ROUTINES  **************************
void incrementEncoder0() {
  if (digitalRead(encoder0PinA) == digitalRead(encoder0PinB)) {
    encoder0Pos++;
  } else {
    encoder0Pos--;
  }
  Serial.println ("Enc0 pos:");
  Serial.println (encoder0Pos, DEC);
}

void incrementEncoder1() {
  if (digitalRead(encoder1PinA) == digitalRead(encoder1PinB)) {
    encoder1Pos++;
  } else {
    encoder1Pos--;
  }
  Serial.println ("Enc1 pos:");
  Serial.println (encoder1Pos, DEC);
}

//**************************************   FUNCTIONS  ************************************
void M1step1(double stepTime){
    digitalWrite(inA1, LOW);
    digitalWrite(inA2, HIGH);
    digitalWrite(inB1, LOW);
    digitalWrite(inB2, HIGH);
    delay(stepTime);
    }
    
void M1step2(double stepTime){
    digitalWrite(inA1, LOW);
    digitalWrite(inA2, HIGH);
    digitalWrite(inB1, HIGH);
    digitalWrite(inB2, LOW);
    delay(stepTime);
    }

void M1step3(double stepTime){
    digitalWrite(inA1, HIGH);
    digitalWrite(inA2, LOW);
    digitalWrite(inB1, HIGH);
    digitalWrite(inB2, LOW);
    delay(stepTime);
    }
    
void M1step4(double stepTime){
    digitalWrite(inA1, HIGH);
    digitalWrite(inA2, LOW);
    digitalWrite(inB1, LOW);
    digitalWrite(inB2, HIGH);
    delay(stepTime);
    }
    
void M2step1(double stepTime){
    digitalWrite(inC1, LOW);
    digitalWrite(inC2, HIGH);
    digitalWrite(inD1, LOW);
    digitalWrite(inD2, HIGH);
    delay(stepTime);
    }
    
void M2step2(double stepTime){
    digitalWrite(inC1, LOW);
    digitalWrite(inC2, HIGH);
    digitalWrite(inD1, HIGH);
    digitalWrite(inD2, LOW);
    delay(stepTime);
    }

void M2step3(double stepTime){
    digitalWrite(inC1, HIGH);
    digitalWrite(inC2, LOW);
    digitalWrite(inD1, HIGH);
    digitalWrite(inD2, LOW);
    delay(stepTime);
    }
    
void M2step4(double stepTime){
    digitalWrite(inC1, HIGH);
    digitalWrite(inC2, LOW);
    digitalWrite(inD1, LOW);
    digitalWrite(inD2, HIGH);
    delay(stepTime);
    }

void M1stop(){
    digitalWrite(inA1, LOW);
    digitalWrite(inA2, LOW);
    digitalWrite(inB1, LOW);
    digitalWrite(inB2, LOW);
    }

void M2stop(){
    digitalWrite(inC1, LOW);
    digitalWrite(inC2, LOW);
    digitalWrite(inD1, LOW);
    digitalWrite(inD2, LOW);
    }
    
//Extend and retract functions move actuator 0.064mm in each direction
void M1extend(double stepTime){
        M1step1(stepTime);
        M1step2(stepTime);
        M1step3(stepTime);
        M1step4(stepTime);
    }
    
void M1retract(double stepTime){
        M1step3(stepTime);
        M1step2(stepTime);
        M1step1(stepTime);
        M1step4(stepTime);
    }
    
void M2extend(double stepTime){
        M2step1(stepTime);
        M2step2(stepTime);
        M2step3(stepTime);
        M2step4(stepTime);
    }
    
void M2retract(double stepTime){
        M2step3(stepTime);
        M2step2(stepTime);
        M2step1(stepTime);
        M2step4(stepTime);
    }
    
void M12set(double m1target, double m2target){
        double stepTime = 3;         //Set dual motor delay
        while (((m1pos - m1target) < 0.07) && ((m2pos - m2target) < 0.07)){
            M1step1(stepTime);
            M2step1(stepTime);
            M1step2(stepTime);
            M2step2(stepTime);
            M1step3(stepTime);
            M2step3(stepTime);
            M1step4(stepTime);
            M2step4(stepTime);
            
            m1pos += 0.064;
            m2pos += 0.064;         
        }
        
        while (((m1pos - m1target) < 0.07) && ((m2pos - m2target) > 0.07)){
            M1step1(stepTime);
            M2step3(stepTime);
            M1step2(stepTime);
            M2step2(stepTime);
            M1step3(stepTime);
            M2step1(stepTime);
            M1step4(stepTime);
            M2step4(stepTime);
        
            m1pos += 0.064;
            m2pos -= 0.064;
        }
        
        while (((m1pos - m1target) > 0.07) && ((m2pos - m2target) < 0.07)){
            M1step3(stepTime);
            M2step1(stepTime); 
            M1step2(stepTime);
            M2step2(stepTime);
            M1step1(stepTime);
            M2step3(stepTime);
            M1step4(stepTime);
            M2step4(stepTime); 
        
            m1pos -= 0.064;
            m2pos += 0.064;   
        }
        
        while (((m1pos - m1target) > 0.07) && ((m2pos - m2target) > 0.07)){
            M1step3(stepTime);
            M2step3(stepTime);
            M1step2(stepTime);
            M2step2(stepTime);
            M1step1(stepTime);
            M2step1(stepTime);
            M1step4(stepTime);
            M2step4(stepTime);
            
            m1pos -= 0.064;
            m2pos -= 0.064;   
        }
        
        //stepTime = 2;            //Set single motor delay
        while ((m1pos - m1target) < 0.07){
            M1step1(stepTime);
            M1step2(stepTime);
            M1step3(stepTime);
            M1step4(stepTime);    //Extend M1 0.064mm
            m1pos += 0.064;
        }
        
        while ((m1pos - m1target > 0.07)){
            M1step3(stepTime);
            M1step2(stepTime);
            M1step1(stepTime);
            M1step4(stepTime);    //Retract M1 0.064mm
            m1pos -= 0.064;
        }
        
        while ((m2pos - m2target) < 0.07){
            M2step1(stepTime);
            M2step2(stepTime);
            M2step3(stepTime);
            M2step4(stepTime);    //Retract M1 0.064mm
            m2pos += 0.064;
        }
        
        while ((m2pos - m2target) > 0.07){
            M2step3(stepTime);
            M2step2(stepTime);
            M2step1(stepTime);
            M2step4(stepTime);    //Extend M1 0.064mm
            m2pos -= 0.064;
        }
        
        M1stop();
        M2stop();
    }
    
void M12home(){
  Serial.println("Home begin");
  for(int s=0 ; s<300 ; s++){
    M1retract(3);
    M1stop();
  }
  for(int s=0 ; s<300 ; s++){

    M2retract(3);
    M1stop();
  }
  M1stop();
  M2stop();
  
  m1pos=0;
  m2pos=0;

  Serial.println("Home end");
}

double checkSpeedLeft(){
  double period = 500;
  encoder0Pos = 0;
  
  //EIFR = bit (INTF0);
  interrupts();
  delay(period);                  //Wait while we collect interrupts
  noInterrupts();

  //Get speed from encoder position, 400 IS 'CLICKS' PER ROTATION
  double angle = (double(encoder0Pos)/400.0) * 360.0;
  return (angle / (period/1000));
}

double checkSpeedRight(){
  double period = 500;
  encoder1Pos = 0;
  
  //EIFR = bit (INTF1);
  interrupts();
  delay(period);                  //Wait while we collect interrupts
  noInterrupts();

  //Get speed from encoder position, 400 IS 'CLICKS' PER ROTATION
  double angle = (double(encoder1Pos)/400.0) * 360.0;
  return (angle / (period/1000));
}

void LEDs(double left, double right){
  //LEDs 0-5 are left, 6-11 are right
  digitalWrite(LED0, LOW);
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);
  digitalWrite(LED3, LOW);
  digitalWrite(LED4, LOW);
  digitalWrite(LED5, LOW);
  digitalWrite(LED6, LOW);
  digitalWrite(LED7, LOW);
  digitalWrite(LED8, LOW);
  digitalWrite(LED9, LOW);
  digitalWrite(LED10, LOW);
  digitalWrite(LED11, LOW);

  //Set left LEDs
  if ((left < 0) && (left > 0.16)){
    digitalWrite(LED0, HIGH);
  }
  if (left < 0.16){
    digitalWrite(LED1, HIGH);
  }
  if (left < 0.33){
    digitalWrite(LED2, HIGH);
  }
  if (left < 0.5){
    digitalWrite(LED3, HIGH);
  }
  if (left < 0.66){
    digitalWrite(LED4, HIGH);
  }
  if (left < 0.83){
    digitalWrite(LED5, HIGH);
  }

  //Set right LEDs
  if ((right < 0) && (left > 0.16)){
    digitalWrite(LED11, HIGH);
  }
  if (right < 0.16){
    digitalWrite(LED10, HIGH);
  }
  if (right < 0.33){
    digitalWrite(LED9, HIGH);
  }
  if (right < 0.5){
    digitalWrite(LED8, HIGH);
  }
  if (right < 0.66){
    digitalWrite(LED7, HIGH);
  }
  if (right < 0.83){
    digitalWrite(LED6, HIGH);
  }
  
}

double activationLeft(double fullActivationBoundary, double partialActivationBoundary, double distanceYellow, double distanceGreen)
{
  //want this to be more sensitive the front yellow by a factor of 2
  double distanceGreenSensitivity;
  distanceGreenSensitivity = distanceGreen*2;
  double activateLeft;

  if ( (distanceYellow<=fullActivationBoundary) || (distanceGreenSensitivity <=fullActivationBoundary))
  {
    activateLeft = 1.0;
  }

  if ( ((distanceYellow>=fullActivationBoundary) && (distanceYellow<=partialActivationBoundary)) || ((distanceGreenSensitivity>=fullActivationBoundary) && (distanceGreenSensitivity<=partialActivationBoundary)))
  {
    if(distanceYellow<distanceGreenSensitivity)
    {
      activateLeft = linearRelationship(distanceYellow,partialActivationBoundary,fullActivationBoundary);
    }
    if(distanceYellow>distanceGreenSensitivity)
    {
      activateLeft = linearRelationship(distanceGreenSensitivity,partialActivationBoundary,fullActivationBoundary);
    }
  }

  return activateLeft;
  
}

double activationRight(double fullActivationBoundary, double partialActivationBoundary, double distanceStripy, double distanceBlue)
{
  //want this to be more sensitive the front yellow by a factor of 2
  double distanceBlueSensitivity;
  distanceBlueSensitivity = distanceBlue*2;
  double activateRight;
  
  if ( (distanceStripy<=fullActivationBoundary) || (distanceBlueSensitivity <=fullActivationBoundary))
  {
      activateRight = 1.0;
  }

  if ( ((distanceStripy>=fullActivationBoundary) && (distanceStripy<=partialActivationBoundary)) || ((distanceBlueSensitivity>=fullActivationBoundary) && (distanceBlueSensitivity<=partialActivationBoundary)))
  {
    if(distanceStripy<distanceBlueSensitivity)
    {
      activateRight = linearRelationship(distanceStripy,partialActivationBoundary,fullActivationBoundary);
    }
    if(distanceStripy>distanceBlueSensitivity)
    {
      activateRight = linearRelationship(distanceBlueSensitivity,partialActivationBoundary,fullActivationBoundary);
    }
  }

  return activateRight;

}

double linearRelationship(double distance,double x1, double x2)
{
  double percentActivation = 1-((distance - x2)/(x1-x2));
  return percentActivation;
}

double quadraticRelationship(double distance,double x1, double x2)
{
  double percentActivation = pow((1-((distance - x2)/(x1-x2))),2);
  return percentActivation;
}

double sqrtRelationship(double distance,double x1, double x2)
{
  double percentActivation = sqrt(1-((distance - x2)/(x1-x2)));
  return percentActivation;
}

double distanceBlueSensor(double voltage)
{
  double e = 2.718;
  double distance = 547.5 * pow(e,(-4.451*voltage)) + 29.19 * pow(e,(-0.4582*voltage));
  if(distance<10)
  {
    return 0;
  }
  if(distance>80)
  {
    return distance = 100;
  }
  else
  {
    return distance;
  }
} 

double distanceStripySensor(double voltage)
{
  double e = 2.718;
  double distance = 223.6 * pow(e,(-4.424*voltage)) + 41.66 * pow(e,(-0.6203*voltage));
  if(distance<10)
  {
    return 0;
  }
  if(distance>80)
  {
    return distance = 100;
  }
  else
  {
    return distance;
  }
} 

double distanceYellowSensor(double voltage)
{
  double e = 2.718;
  double distance = 462.4 * pow(e,(-5.733*voltage)) + 48.76 * pow(e,(-0.7032*voltage));
  if(distance<10)
  {
    return 0;
  }
  if(distance>80)
  {
    return distance = 100;
  }
  else
  {
    return distance;
  }
} 

double distanceGreenSensor(double voltage)
{
  double e = 2.718;
  double distance = 228.2 * pow(e,(-4.4*voltage)) + 38.9 * pow(e,(-0.584*voltage));
  if(distance<10)
  {
    return 0;
  }
  if(distance>80)
  {
    return distance = 100;
  }
  else
  {
    return distance;
  }
} 


//*******************************  SETUP AND LOOP  ***********************************
void setup() {
  pinMode(encoder0PinA, INPUT_PULLUP);
  pinMode(encoder0PinB, INPUT_PULLUP);
  pinMode(encoder1PinA, INPUT_PULLUP);
  pinMode(encoder1PinB, INPUT_PULLUP);
  
  pinMode(inA1, OUTPUT);
  pinMode(inA2, OUTPUT);
  pinMode(inB1, OUTPUT);
  pinMode(inB2, OUTPUT);
  pinMode(inC1, OUTPUT);
  pinMode(inC2, OUTPUT);
  pinMode(inD1, OUTPUT);
  pinMode(inD2, OUTPUT);
  
  //attachInterrupt(digitalPinToInterrupt(encoder0PinA), incrementEncoder0, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(encoder1PinA), incrementEncoder1, CHANGE);
  
  Serial.begin (9600);
  Serial.println("start");

  M12home();
}

void loop() {
  double targetLeftSpd, targetRightSpd, currentLeftSpd, currentRightSpd, leftBrakePos, rightBrakePos;

  //TODO: Select minumum target left and right speeds
  //Obstacle aviodance routine
  double distanceBlue, distanceStripy,distanceYellow, distanceGreen,partialActivationBoundary,fullActivationBoundary;
  double tempBlue, tempStripy, tempYellow, tempGreen;
  double voltageBlue, voltageStripy, voltageYellow, voltageGreen;
  double activateLeft, activateRight;
  fullActivationBoundary = 15;//must be greater than or equal to 10
  partialActivationBoundary = 75;//must be less than or equal to 80

  tempBlue = analogRead(0);
  voltageBlue = tempBlue * 5 / 1024;
  distanceBlue = distanceBlueSensor(voltageBlue);
  
  tempStripy = analogRead(1);
  voltageStripy = tempStripy * 5 / 1024;
  distanceStripy = distanceStripySensor(voltageStripy);
  
  tempYellow = analogRead(2);
  voltageYellow = tempYellow * 5 / 1024;
  distanceYellow = distanceYellowSensor(voltageYellow);
  
  tempGreen = analogRead(3);
  voltageGreen = tempGreen * 5 / 1024;
  distanceGreen = distanceGreenSensor(voltageGreen);
  
  activateLeft = activationLeft(fullActivationBoundary, partialActivationBoundary, distanceYellow, distanceGreen);
  activateRight = activationRight(fullActivationBoundary, partialActivationBoundary, distanceStripy, distanceBlue);

  //Check for a fall detection
  double noFall = 1.0;
  //if (Serial.read() != 0){
  //  noFall = 0.0;
  //}

  leftBrakePos = min(activateLeft, noFall);
  rightBrakePos = min(activateRight, noFall);

  //Show brake setting on LED indicator
  LEDs(leftBrakePos, rightBrakePos);

  //Scale brake activations by actuator space
  leftBrakePos *= 8;
  rightBrakePos *= 8;
  Serial.println ("left brake pos:");
  Serial.println (leftBrakePos, DEC);
  Serial.println ("right brake pos:");
  Serial.println (rightBrakePos, DEC);
  
  if ((leftBrakePos < 0.2) && (rightBrakePos < 0.2)){
    M12home();
  }
  else{
    M12set(leftBrakePos, rightBrakePos);
  }
  

}
