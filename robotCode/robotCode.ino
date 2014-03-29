#include <FlameSensor.h>
#include <Servo.h>
#include <Ultrasonic.h>


Servo leftServo;
Servo rightServo;
Servo flameServo;

#define flameSensor 14

FlameSensor flame(flameSensor);

#define leftTrig 33
#define leftEcho 32
#define rightTrig 23
#define rightEcho 22
#define frontTrig 25
#define frontEcho 24

Ultrasonic leftSonar( leftTrig , leftEcho );
Ultrasonic rightSonar( rightTrig , rightEcho );
Ultrasonic frontSonar( frontTrig , frontEcho );

#define FORWARDLEFT 1
#define FORWARDRIGHT 2
#define TURNLEFT 3
#define TURNRIGHT 4
#define DECISION 5
#define LOST 6
#define FLAMESEARCH 7

int currentState = NULL;
int lastState = NULL;

#define forwardLeftInd 42
#define forwardRightInd 38 
#define leftTurnInd 46
#define rightTurnInd 44
#define decisionInd 40
#define flameInd 37

#define onButton 48

void extinguishFlame(){

  leftServo.write(80);
  rightServo.write(102);  

  int flameFound = 0;
  while(!flameFound){ 
    for(int angle = 130; angle > 80; angle -= 10){
      if(analogRead(14) > 950){
        flameFound = 1;
        break;
      }
      flameServo.write(angle);
      delay(40);
    }

    for(int angle = 80; angle > 130; angle += 10){
      if(analogRead(14) > 950){
        flameFound = 1;
        break;
      }
      flameServo.write(angle);
      delay(40);
    }

  }
  leftServo.write(90);
  rightServo.write(90);
  digitalWrite(flameInd , HIGH);
}

void detectFlame(){
  int angle = 130;
  flameServo.write(angle);

  while(angle > 80){
    int time = 0;    
    while(time < 1000){      
      leftServo.write(15);
      rightServo.write(15);
      if(analogRead(14) > 400){ 
        leftServo.write(90);
        rightServo.write(90);
        extinguishFlame();
        return;
      } 
      delay(5);
      time += 5;
    }     
    angle = angle - 10;
    flameServo.write(angle);
    time = 0;
    while(time < 1000){
      leftServo.write(165);
      rightServo.write(165);
      if(analogRead(14) > 400){
        leftServo.write(90);
        rightServo.write(90);
        extinguishFlame();
        return;
      } 
      delay(5);
      time += 5;
    }       
    angle = angle - 10;
    flameServo.write(angle);  
    delay(100);
  }
  leftServo.write(90);
  rightServo.write(90);  
}

int pollSonar(Ultrasonic testSonar){
  delay(50);
  return testSonar.convert(testSonar.timing(), 1);
}

int detectCollision(){
  return digitalRead(29);
}

void rightTurn(){
  leftServo.write(70);
  rightServo.write(70);
  delay(1075);
  leftServo.write(90);
  rightServo.write(90);
}

void leftTurn(){
  leftServo.write(110);
  rightServo.write(110);
  delay(1085);
  leftServo.write(90);
  rightServo.write(90);
}

void aboutFace(){
  leftServo.write(110);
  rightServo.write(110);
  delay(2100);
  leftServo.write(50);
  rightServo.write(130);  
}

void noMansLand(){
  leftServo.write(20);
  rightServo.write(160);
  delay(1000);
  rightServo.write(90);
  leftServo.write(90);
}

void allignLeft(){
  while( ((pollSonar(leftSonar) > 8) || (pollSonar(leftSonar) < 7)) && detectCollision() ){
    if( pollSonar(leftSonar) < 7 ){
      rightServo.write(95);
      delay(50);
      rightServo.write(174);
    }
    if( pollSonar(leftSonar) > 8 ){ 
      leftServo.write(85);
      delay(50);
      leftServo.write(20);
    }
  }
}

void allignRight(){
  while( ((pollSonar(rightSonar) > 8) || (pollSonar(rightSonar) < 7)) && detectCollision() ){
    if( pollSonar(rightSonar) < 7 ){
      rightServo.write(95);
      delay(50);
      rightServo.write(174);
    }
    if( pollSonar(rightSonar) > 8 ){ 
      leftServo.write(85);
      delay(50);
      leftServo.write(20);
    }
  }
}
void forwardLeft(){
  while(1){
    leftServo.write(10);
    rightServo.write(174);    
    allignLeft();
    delay(50);
    if( !detectCollision() || (pollSonar(frontSonar) < 10) ){
      leftServo.write(90);
      rightServo.write(90);
      currentState = DECISION;
      return;
    }
    if (pollSonar(leftSonar) > 20){
      delay(1200);
      currentState = DECISION;
      return;
    }
    Serial.println(pollSonar(leftSonar));
  }
}  

void forwardRight(){
  while(1){
    leftServo.write(20);
    rightServo.write(164);
    allignRight();
    delay(50);
    if( !detectCollision() || (pollSonar(frontSonar) < 10) ){
      currentState = DECISION;
      return;
    }
    if (pollSonar(rightSonar) > 30){
      delay(1200);
      currentState = DECISION;
      return;
    }
  }
}  

void makeDecision(){
  //front wall and open left wall and closed right wall, TURNRIGHT, make new decision
  if( !detectCollision() && (pollSonar(leftSonar) > 30) && (pollSonar(rightSonar) < 15) ){
    currentState = TURNLEFT;
    return;
  }

  //front wall and open right wall and closed left wall, TURNLEFT, make new decision
  if( !detectCollision() && (pollSonar(rightSonar) > 30) && (pollSonar(leftSonar) < 15) ){
    currentState = TURNRIGHT;
    return;
  }

  if( detectCollision() && (pollSonar(rightSonar) > 20) && (pollSonar(leftSonar) < 20)){
    currentState = FORWARDLEFT;
    return;
  }

  if( (lastState == FORWARDLEFT) && (pollSonar(leftSonar) > 25) ){
    leftTurn();
    int time = 0;
    
    //keep from crashing into a wall while moving around doorways
    while( time < 75 ){
      leftServo.write(50);
      rightServo.write(134);    
      if( pollSonar(leftSonar) < 6 ){
        leftServo.write(90);
        rightServo.write(70);
      }
      if( pollSonar(rightSonar) < 6){
        leftServo.write(110);
        rightServo.write(90);
      }

      time += 5;
      delay(5);
    }

    currentState = DECISION;
    return;
  }

  if( (lastState == FORWARDRIGHT) && (pollSonar(rightSonar) > 25) ){
    rightTurn();
    int time = 0;
    
    //keep from crashing into a wall while moving around doorways
    while( time < 75 ){
      leftServo.write(50);
      rightServo.write(134);    
      if( pollSonar(leftSonar) < 6 ){
        leftServo.write(90);
        rightServo.write(70);
      }
      if( pollSonar(rightSonar) < 6){
        leftServo.write(110);
        rightServo.write(90);
      }

      time += 5;
      delay(5);
    }
    
    currentState = DECISION;
    return;
  }  
}

void navigate(){ 
  while(1){
    switch(currentState){
    case FORWARDLEFT: 
      digitalWrite(forwardLeftInd, HIGH);
      forwardLeft();
      lastState = FORWARDLEFT;
      digitalWrite(forwardLeftInd, LOW);
      break;
    case FORWARDRIGHT:
      digitalWrite(forwardRightInd, HIGH);
      forwardRight();
      lastState = FORWARDRIGHT;
      digitalWrite(forwardRightInd, LOW);
      break;        
    case TURNLEFT:
      digitalWrite(leftTurnInd, HIGH);
      leftTurn();
      currentState = FORWARDRIGHT;
      digitalWrite(leftTurnInd, LOW);       
      break; 
    case TURNRIGHT:
      digitalWrite(rightTurnInd, HIGH);
      rightTurn();
      currentState = FORWARDLEFT;
      digitalWrite(rightTurnInd, LOW);       
      break;     
    case DECISION:
      digitalWrite(decisionInd, HIGH);
      leftServo.write(90);
      rightServo.write(90);
      makeDecision();
      digitalWrite(decisionInd, LOW);
      break;
    case LOST:
      noMansLand();
      digitalWrite(decisionInd, LOW);
      currentState = DECISION;
      break;

    }   
  }
}

void setup(){

  pinMode(onButton, INPUT);

  pinMode(leftTrig, OUTPUT);
  pinMode(rightTrig, OUTPUT);
  pinMode(frontTrig, OUTPUT);
  pinMode(leftEcho, INPUT);
  pinMode(rightEcho, INPUT);
  pinMode(frontEcho, INPUT);

  pinMode(forwardLeftInd, OUTPUT);
  pinMode(forwardRightInd, OUTPUT);
  pinMode(leftTurnInd, OUTPUT);
  pinMode(rightTurnInd, OUTPUT);
  pinMode(decisionInd, OUTPUT);
  pinMode(flameInd, OUTPUT);

  pinMode(flameSensor, INPUT);

  Serial.begin(9600);

  while(!digitalRead(48)){}
  leftServo.attach(53);
  rightServo.attach(52);
  flameServo.attach(51);  
  delay(3000);

  currentState = FORWARDLEFT;
  navigate();


}

void loop(){}
