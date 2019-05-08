#include <Pixy2.h>
#include <Servo.h>
#include "Adafruit_APDS9960.h"

Pixy2 forwardCam;
Servo mitt;
Servo steer;
Adafruit_APDS9960 apds;

enum state {
  TRACK,
  FIELD,
  RUN,
  TAG,
  END
};

int photoDiode = 12;
int motor = 3;
int proximity = 7;
int objX = 158;
int err = 0;
int loops;
int errorDerivative[5];
bool sawBall;
int closestBase;
int seen;

// PID variables for steering //
double steerP, steerI, steerD;
double steerKP, steerKI, steerKD; 

state currState;

void setup() {
  // put your setup code here, to run once:
  // Attaching motors/servos/sensors to correct pin
  pinMode(photoDiode, INPUT);
  pinMode(motor, OUTPUT);
  pinMode(proximity, INPUT_PULLUP);
  mitt.attach(9);
  mitt.write(5);
  steer.attach(10);
  steer.write(90);
  Serial.begin(115200);
  
  if(!apds.begin()) {
    Serial.println("Couldn't initialize APDS");
  }
  else Serial.println("APDS initialized!"); 
  forwardCam.init();
  sawBall = false;
  loops = 0;
  seen = 0;
  currState = TRACK;
  steerKP = 0.8; steerKI = 0; steerKD = 0;
  analogWrite(motor, 80);
  
  apds.enableProximity(true);
  apds.setProximityInterruptThreshold(0, 3);

  //enable the proximity interrupt
  apds.enableProximityInterrupt();
}

void doSteer(double P, double I, double D) {
  int toServo = (int) 90 + P + I + D;
  if (toServo < 0) toServo = 0;
  else if (toServo > 180) toServo = 180;
  steer.write(toServo);
}

void turnAround(){
  steer.write(60);

  analogWrite(motor, 31);
}

void getDeriv() {
  int i = loops % 5;
  errorDerivative[i] = err;
  if (loops >= 5) {
    if (i == 0) {
      steerD = steerKD * ((errorDerivative[i] - errorDerivative[i+4]) / 5.0);
    }
    else {
      steerD = steerKD * ((errorDerivative[i] - errorDerivative[i-1]) / 5.0);
    }  
  }
  else steerD = 0; // Don't start derivative term until more than 5 loops
}

void liftArm() {
  mitt.write(55);
  Serial.println("Lifted");
  return;
}

void lowerArm() {
  mitt.write(5);
  Serial.println("Lowered");
  return;
}

void loop() {


  /*while (true) {
    Serial.print("PRoximity");
    Serial.println(apds.readProximity());
    delay(333);
  } */
  // put your main code here, to run repeatedly:
  //Blocks ball[]; // ball block
  //Blocks bases[]; // bases block
  switch(currState) {
    case TRACK: {
      
      //Color code based off Adafruit example
      //create some variables to store the color data in
      if(!digitalRead(proximity)){
        Serial.println(apds.readProximity());
        currState = FIELD;

        //clear the interrupt
        apds.clearInterrupt();
        break;
      }
      
      // Get the error for PID (based on x-location of ball in frame)
      forwardCam.ccc.getBlocks(1);
      Serial.print("Blocks: ");
      Serial.println(forwardCam.ccc.numBlocks);
      if (forwardCam.ccc.numBlocks > 0) {
        //sawBall = true;
        objX = forwardCam.ccc.blocks[0].m_x;
        /*if(forwardCam.ccc.blocks[0].m_y > 190){
          steer.write(90);
          delay(500);
          currState = FIELD;
          Serial.println("Ball moved out of frame, going to field.");
          break;
        }
        else */ err = objX - 158;
      }
      /*else if (sawBall) {
          steer.write(90);
          delay(400);
          currState = FIELD;
          Serial.println("Ball moved out of frame, going to field.");
          break;
      }
      */
      else err = 0;
    
      // Calculations for PID components
      getDeriv(); // derivative (sets global variable)
      loops++; 
      steerI += steerKI * err; // integral
      steerP = steerKP * err; // proportional

      doSteer(steerP, steerI, steerD); // Do actual steer
      break;
    }
    case FIELD: {
      analogWrite(motor, 0);
      liftArm();
      currState = RUN;
      steerP, steerI, steerD = 0;
      loops = 0;
      delay(2000);
      Serial.println("Finished fielding, going to 'run'");
      break;
    }
    case RUN: {
        Serial.print("Err: ");
        Serial.println(err);

      forwardCam.ccc.getBlocks(false, 14); // we want signatures 11110, just the bases
      // the array will be automatically ordered with the largest object first

      if (forwardCam.ccc.numBlocks == 0 && !seen) {
        turnAround();
        Serial.println("Didn't see base, so turning until one found.");
        break;
      }
      else {
        Serial.println("Saw base");
        analogWrite(motor, 50);
      }

      if (seen == 0) {
        closestBase = forwardCam.ccc.blocks[0].m_index;
        liftArm();
        seen = 1;
      }
      
      for(int i = 0; i < forwardCam.ccc.numBlocks; i++){
          if (forwardCam.ccc.blocks[i].m_index == closestBase){
            
            objX = forwardCam.ccc.blocks[i].m_x;

            err = objX - 158;
            if (forwardCam.ccc.blocks[i].m_width > 75){
              Serial.println("Base large in frame, moving to tag.");
              analogWrite(motor, 0);
              currState = TAG;
              break;
            }
            else if (forwardCam.ccc.blocks[i].m_width > 50) {
              analogWrite(motor, 35);
            }
            getDeriv(); // derivative (sets global variable)
            steerI += steerKI * err; // integral
            steerP = steerKP * err; // proportional

            doSteer(steerP, steerI, steerD); // Do actual steer
            
            }
            
          else if (i == (forwardCam.ccc.numBlocks - 1)) {
            closestBase = forwardCam.ccc.blocks[0].m_index;
            /*doSteer(0,0,0); */ 
          }
        }
        loops++;
        break;
    }

    case TAG: {
      lowerArm();
      currState = END;
      Serial.println("Lowered arm, now ending.");
      //make it go back to home base?
      //forwardCam.ccc.getBlocks(32);
      break;
    }
    case END: { 
      mitt.detach();
      steer.detach();
      analogWrite(motor, 0);
    }
    }
  // program to lift the arm using photoDiode
  // detect 
  delay(5);
}
