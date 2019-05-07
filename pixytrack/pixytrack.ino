#include <Pixy2.h>
#include <Servo.h>
//#include "Adafruit_APDS9960.h"

Pixy2 forwardCam;
Servo mitt;
Servo steer;
//Adafruit_APDS9960 apds;

enum state {
  TRACK,
  FIELD,
  RUN,
  TAG,
  END
};

int photoDiode = 12;
int motor = 3;
//int proximity = 7;
int objX = 168;
int err = 0;
int armPos = 135;
int loops;
int errorDerivative[5];
bool sawBall;

// PID variables for steering //
double steerP, steerI, steerD;
double steerKP, steerKI, steerKD; 

state currState;

void setup() {
  // put your setup code here, to run once:
  // Attaching motors/servos/sensors to correct pin
  pinMode(photoDiode, INPUT);
  pinMode(motor, OUTPUT);
  //pinMode(proximity, INPUT_PULLUP);
  mitt.attach(9);
  steer.attach(10);
  Serial.begin(115200);
  
  /*if(!apds.begin()) {
    Serial.println("Couldn't initialize APDS");
  }
  else Serial.println("APDS initialized!"); 
  */
  forwardCam.init();
  sawBall = false;
  mitt.write(140);
  steer.write(90);
  loops = 0;
  currState = TRACK;
  steerKP = 0.7; steerKI = 0; steerKD = 0.05;
  analogWrite(motor, 80);
  
  /*apds.enableProximity(true);
  apds.setProximityInterruptThreshold(0, 100);

  //enable the proximity interrupt
  apds.enableProximityInterrupt(); */
}

void doSteer(double P, double I, double D) {
  int toServo = (int) 90 + P + I + D;
  if (toServo < 0) toServo = 0;
  else if (toServo > 180) toServo = 180;
  steer.write(toServo);
}

void turnAround(){
  steer.write(0);
  analogWrite(motor, 60);
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
  mitt.write(180);
  Serial.print("Lifted");
  return;
}

void lowerArm() {
  mitt.write(135);
  Serial.print("Lowered");
  return;
}

void loop() {
  // put your main code here, to run repeatedly:
  //Blocks ball[]; // ball block
  //Blocks bases[]; // bases block
  switch(currState) {
    case TRACK: {
      
      //Color code based off Adafruit example
      //create some variables to store the color data in
      /* if(!digitalRead(proximity)){
        Serial.println(apds.readProximity());
        currState = FIELD;

        //clear the interrupt
        apds.clearInterrupt();
        break;
      } */
      
      if (loops >= 200) {
        Serial.println("End of loops, going to field.");
        currState = FIELD;
        break;
      }
      // Get the error for PID (based on x-location of ball in frame)
      forwardCam.ccc.getBlocks(1);
      Serial.print("Blocks: ");
      Serial.println(forwardCam.ccc.numBlocks);
      if (forwardCam.ccc.numBlocks > 0) {
        sawBall = true;
        objX = forwardCam.ccc.blocks[0].m_x;
      
        err = objX - 158;
      }
      else if (sawBall) {
          delay(20);
          currState = FIELD;
          Serial.println("Ball moved out of frame, going to field.");
          break;
      }
      else {err = 0;}
      

      /* ! THIS SHOULD PROBABLY BE A WHILE LOOP ! */
      /*for (int i = 0; i < (sizeof(forwardCam.ccc.blocks) / sizeof(forwardCam.ccc.blocks[0])); i++) {
        if (forwardCam.ccc.blocks[i].m_width < 5) continue; // Avoid false positives
        objX = forwardCam.ccc.blocks[i].m_x;
        err = objX - 168;
      } */
    
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
      delay(250);
      Serial.println("Finished fielding, going to 'run'");
      break;
    }
    case RUN: {
      
      forwardCam.ccc.getBlocks(30); // we want signatures 11110, just the bases
      // the array will be automatically ordered with the largest object first
      if (forwardCam.ccc.numBlocks == 0) {
        turnAround();
        Serial.println("Didn't see base, so turning until one found.");
        break;
      }

      else {
        Serial.println("Saw base");
        analogWrite(motor, 75);
        forwardCam.ccc.blocks[0];
        objX = forwardCam.ccc.blocks[0].m_x;

        err = objX - 158;

        if (forwardCam.ccc.blocks[0].m_width > 125){
          Serial.println("Base large in frame, moving to tag.");
          currState = TAG;
          break;
        }
      }

      getDeriv(); // derivative (sets global variable)
      steerI += steerKI * err; // integral
      steerP = steerKP * err; // proportional

      doSteer(steerP, steerI, steerD); // Do actual steer
      break;

    }
    case TAG: {
      lowerArm();
      //turnAround();
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
