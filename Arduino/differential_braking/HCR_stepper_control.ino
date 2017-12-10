int inA1 = 6;
int inA2 = 7;
int inB1 = 8;
int inB2 = 9;
int inC1 = 10;
int inC2 = 11;
int inD1 = 12;
int inD2 = 13;

double m1pos, m2pos;

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

void M2home(){
  Serial.println("Home begin");
  for(int s=0 ; s<300 ; s++){

    M2retract(3);
    M2stop();
  }
  M2stop();
  m2pos=0;

  Serial.println("Home end");
}

int checkSerial(){
  if(Serial.available() > 0){
    int fallTarget = Serial.read();
  }
}

void setup() {
  Serial.begin(9600);
  
  //Home both motors to start routine
  pinMode(inA1, OUTPUT);
  pinMode(inA2, OUTPUT);
  pinMode(inB1, OUTPUT);
  pinMode(inB2, OUTPUT);
  pinMode(inC1, OUTPUT);
  pinMode(inC2, OUTPUT);
  pinMode(inD1, OUTPUT);
  pinMode(inD2, OUTPUT);
  
  M12home();
  M12set(10,10);
  //delay(3000);
  //M12home();
  
}

void loop() {
  //M12set(10,10);
  //M12set(0,0);
}









