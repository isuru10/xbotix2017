#include <QTRSensors.h>
#include <Servo.h>

#define Kp 0.2                    // experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
#define Kd 1                    // experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd) 
#define MaxSpeed 150              // max speed of the robot
#define BaseSpeed 100            // this is the speed at which the motors should spin when the robot is perfectly on the line
#define NUM_SENSORS  8            // number of sensors used

#define speedwhite 100

#define intersectionspeed 60

#define rightMotor1 36
#define rightMotor2 37
#define rightMotorPWM 12

#define leftMotor1 45
#define leftMotor2 44
#define leftMotorPWM 13
#define motorPower A3
#define buzzer 52
Servo arm;
Servo grip;
int position;
int lastError=0;
int error;
QTRSensorsRC qtrrc((unsigned char[]) {47, 25, 26, 27, 28, 29, 53, 51} ,NUM_SENSORS, 2500, QTR_NO_EMITTER_PIN);

unsigned int sensorValues[NUM_SENSORS];
bool dval[NUM_SENSORS];

void qtrRead()
{
  position = qtrrc.readLine(sensorValues);
  
  for(int i=0 ; i < NUM_SENSORS ; i++)
  {
    if(sensorValues[i] > 600) dval[i]=1;
    else dval[i] = 0;
  }
}

void setup() {
  // put your setup code here, to run once:
  pinMode(rightMotor1,   OUTPUT);
  pinMode(rightMotor2,   OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(leftMotor1,    OUTPUT);
  pinMode(leftMotor2,    OUTPUT);
  pinMode(leftMotorPWM,  OUTPUT);
  pinMode(motorPower,    OUTPUT);
  pinMode(buzzer, OUTPUT);
  arm.attach(22);
  grip.attach(24);
  arm.write(160);
  grip.write(70);
  
  for(int i =0; i <100; i++){
    qtrrc.calibrate();   
    delay(20);
    }

    digitalWrite(buzzer, HIGH);
    delay(500);
    digitalWrite(buzzer, LOW);

    delay(2000);

}

void loop() {
  // put your main code here, to run repeatedly:
  start();
  turnLeft2();
  deadEnd();
  lowerArm();
  //detect
  gripper();
  raiseArm();
  //reverse();
  turnAround();
  TLeft();
  turnRight2();
  turnLeft();
  deadEnd();
  lowerArm();
  loose();
  raiseArm();
  gripper();
  right();
  delay(150);
  turnAround();
  turnRight();
  TRight();
  //
  turnRight();
  turnLeft();
  turnLeft();
  turnRight();
  turnLeft();
  turnRight2();
  deadEnd();
  start();
  extraInch();
  extraInch();
  extraInch();
  turnLeft2();
  deadEnd();
  extraInch();
  reverse();
  right();
  delay(100);
  turnAround();
  
  //
  
  while(1);
}

void deadEnd(){
  while(1){
      qtrRead();
      if(dval[0] && dval [3] && dval[7]){
          brake();
          break;
        }else{
          PID();
          }
    }
  }

  void lowerArm(){
    for(int i = 160; i > 100; i--){
        arm.write(i);
        delay(18);
      }
  }

  void raiseArm(){
    for(int i = 0; i < 160; i++){
        arm.write(i);
        delay(18);
      }
  }

  void gripper(){
    for(int i = 70; i > 0; i--){
        grip.write(i);
        delay(18);
      }
  }

  void loose(){
    for(int i = 0; i < 70; i++){
        grip.write(i);
        delay(18);
      }
    }

void start(){
    while(1){
      forward(BaseSpeed, BaseSpeed);
      qtrRead();
      if(!dval[0] && !dval[7]){
         
          break;
        }else{
          
          }
      
      }
  }

void PID()
{
  int error = 3500 - position  ;
  int motorSpeed = Kp * error + Kd * (error - lastError);
  lastError = error;

  int rightMotorSpeed = BaseSpeed + motorSpeed;
  int leftMotorSpeed = BaseSpeed - motorSpeed;
  
  if (rightMotorSpeed > MaxSpeed ) rightMotorSpeed = MaxSpeed;        // prevent the motor from going beyond max speed
  if (leftMotorSpeed  > MaxSpeed ) leftMotorSpeed = MaxSpeed;         // prevent the motor from going beyond max speed
  if (rightMotorSpeed < 0)rightMotorSpeed = 0;    
  if (leftMotorSpeed  < 0)leftMotorSpeed = 0;
        
  forward(leftMotorSpeed, rightMotorSpeed);
}

void forward(int LSpeed, int RSpeed)
{
  digitalWrite(rightMotor1, HIGH);
  digitalWrite(rightMotor2, LOW);
  digitalWrite(leftMotor1, HIGH);
  digitalWrite(leftMotor2, LOW);

  analogWrite(leftMotorPWM, LSpeed);
  analogWrite(rightMotorPWM, RSpeed);
}

void left(){
  digitalWrite(rightMotor1, HIGH);
  digitalWrite(rightMotor2, LOW);
  digitalWrite(leftMotor1, LOW);
  digitalWrite(leftMotor2, HIGH);

  analogWrite(leftMotorPWM, 110);
  analogWrite(rightMotorPWM, 110);
}


//void left(int LSpeed, int RSpeed){
//  digitalWrite(rightMotor1, HIGH);
//  digitalWrite(rightMotor2, LOW);
//  digitalWrite(leftMotor1, LOW);
//  digitalWrite(leftMotor2, HIGH);

//  analogWrite(leftMotorPWM, LSpeed);
//  analogWrite(rightMotorPWM,RSpeed);
//}

void right(){
  digitalWrite(rightMotor1, LOW);
  digitalWrite(rightMotor2, HIGH);
  digitalWrite(leftMotor1, HIGH);
  digitalWrite(leftMotor2, LOW);

  analogWrite(leftMotorPWM, 110);
  analogWrite(rightMotorPWM, 110);
}

void turnRight(){
    while(1){
           qtrRead();
        if(dval[7]){
            extraInch();
            rightTurn();
            break;
          }else{
           
            }      
      }
  }

  void turnLeft(){
    while(1){
      
      qtrRead();
        if(dval[0]){
            extraInch();
            leftTurn();
            break;
          }else{
            
            }      
      }
  }

  void turnLeft2(){
    while(1){
      qtrRead();
        if(dval[0]){
            extraInch();
            leftWhite();
            leftTurn();
            break;
          }else{
            PID();
            }      
      }
  }

  void turnRight2(){
    while(1){
      qtrRead();
        if(dval[7]){
          brake();
            extraInch();
           rightWhite();
           rightTurn();
            break;
          }else{
            PID();
            }      
      }
  }

  void rightWhite(){
    while(1)
 { 
     right();
    qtrRead();
    
    if(!dval[3] && !dval[4]) 
    {     
      //brake();
      break;
    }
    else{
            right();
        }
  }
    }

  void extraInch(){      
      forward(BaseSpeed,BaseSpeed);
      delay(100);
      brake();
      
  }

  void reverse(){      
      backward();
      delay(100);
      brake();
      
  }

  void rightTurn(){
    while(1)
    {   right();  
      qtrRead();    
    if(dval[3] || dval[4]) 
    {     
      //brake();
      break;
    }
    else{
             right();
        }
  }
  }
void turnAround(){
  while(1){
     digitalWrite(rightMotor1, LOW);
  digitalWrite(rightMotor2, HIGH);
  digitalWrite(leftMotor1, HIGH);
  digitalWrite(leftMotor2, LOW);

  analogWrite(leftMotorPWM, 100);
  analogWrite(rightMotorPWM, 100);
      qtrRead();
      if((!dval[0] && !dval[7]) && (dval[3] || dval[4])){
          brake();
          break;
        }else{
         
          }
    }
  }

  void TLeft(){
    while(1){
        qtrRead();
        if(dval[0] || dval[7]){
          extraInch();
          leftTurn();
          break;
          }else{
            PID();
            }
      }
    }

    void TRight(){
    while(1){
        qtrRead();
        if(dval[0] || dval[7]){
          extraInch();
          rightTurn();
          break;
          }else{
            PID();
            }
      }
    }


     void leftTurn(){
    while(1)
 { 
    left();
    qtrRead();
    
    if(dval[3] || dval[4]) 
    {     
      //brake();
      break;
    }
    else{
             
        }
  }
  }

  

  void leftWhite(){
    while(1)
 { 
     left(); 
    qtrRead();
    
    if(!dval[3] && !dval[4]) 
    {     
      
      break;
    }
    else{
           
        }
  }
  }
  
void backward(){
  digitalWrite(rightMotor1, LOW);
  digitalWrite(rightMotor2, HIGH);
  digitalWrite(leftMotor1, LOW);
  digitalWrite(leftMotor2, HIGH);

  analogWrite(leftMotorPWM, BaseSpeed);
  analogWrite(rightMotorPWM, BaseSpeed);
}



void brake(){
  digitalWrite(rightMotor1, HIGH);
  digitalWrite(rightMotor2, HIGH);
  digitalWrite(leftMotor1, HIGH);
  digitalWrite(leftMotor2, HIGH);
  }

    void hardright(){
    while(1)
 { 
    
    qtrRead();
    
    if(dval[7] && !dval[0]) 
    {     
      turnRight();
      break;
    }
    else{
         PID();    
        }
  }
  }

  void maze(){
    
    
    }

//  //void mesh(){
//     while(1)
// { 
//    qtrRead();
//    if(dval[0]){
//      turnLeft();
//    }
//    else if(dval[7]){
//      turnRight();
//      }  
//     else if(dval[0] && dval[7]){
//      while(1){
//        forward(100,100);
//        if((!dval[0] && !dval[7])&&(!dval[3] || !dval[4])){
//          turnAround();
//          TLeft();
//          forward(100,100);
//          delay(1000);
//          if((!dval[0] && !dval[7])&&(!dval[3] || !dval[4])){
//            turnAround();
//            forward(100,100);
//            delay(1000);
//            }
//            while(1){
//             if(dval[0] && dval[7]){
//              
//              PID();
//              break;
//              }
//           
//          }
//        }
//      }
//
//      
//      break;
//    }
//    
//           
//      
//  }
//  }
//  

