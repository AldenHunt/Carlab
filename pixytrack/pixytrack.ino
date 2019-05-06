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
  TAG
};

int photoDiode = 12;
int motor = 3;
int objX = 168;
int err = 0;
int armPos = 135;
int loops;
int errorDerivative[5];

// PID variables for steering //
double steerP, steerI, steerD;
double steerKP, steerKI, steerKD; 

state currState;

void setup() {
  // put your setup code here, to run once:
  // Attaching motors/servos/sensors to correct pin
  pinMode(photoDiode, INPUT);
  pinMode(motor, OUTPUT);
  mitt.attach(9);
  steer.attach(10);
  Serial.begin(9600);
  forwardCam.init();
  
  mitt.write(140);
  steer.write(90);
  loops = 0;
  currState = TRACK;
  steerKP = 0.7; steerKI = 0; steerKD = 0.05;
  analogWrite(motor, 75);
}

void doSteer(double P, double I, double D) {
  int toServo = (int) 90 + P + I + D;
  if (toServo < 0) toServo = 0;
  else if (toServo > 180) toServo = 180;
  Serial.print("Steer: ");
  Serial.println(toServo);
  steer.write(toServo);
}

void turnAround(){
  steer.write(45);
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
  /*
  // Get the error for PID (based on x-location of ball in frame)
  ball = forwardCam.ccc.getBlocks(true, 1); // wait until it detects object with signature 1 (ball) 
  for (int i = 0; i < (sizeof(forwardCam.ccc.blocks) / sizeof(forwardCam.ccc.blocks[0])); i++) {
    if (forwardCam.ccc.blocks[i].m_width < 5) continue; // Avoid false positives
    objX = forwardCam.ccc.blocks[i].m_x;
    err = objX - 168;
    break;
  } */

  switch(currState) {
    case TRACK: {

      if (loops >= 150) {
        currState = FIELD;
        break;
      }
      // Get the error for PID (based on x-location of ball in frame)
      forwardCam.ccc.getBlocks();
      Serial.print("Blocks: ");
      Serial.println(forwardCam.ccc.numBlocks);
      //forwardCam.ccc.blocks[0].print();
      if (forwardCam.numBlocks > 0) {
        objX = forwardCam.ccc.blocks[0].m_x;
      
        err = objX - 158; /* shouldn't this be 158*/
      }

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
      break;
    }
    case RUN: {

      forwardCam.ccc.getBlocks(14); // we want signatures 1110, just the bases
      /* the array will be automatically ordered with the largest object first*/
      if (forwardCam.ccc.numBlocks == 0) {
        turnAround();
        break;
      /* make the car rotate?*/
      }

      if (forwardCam.ccc.numBlocks > 0) {
        forwardCam.ccc.blocks[0];
        objX = forwardCam.ccc.blocks[0].m_x;

        err = objX - 158;

        if (forwardCam.ccc.blocks[0].width > 250){
          currState = TAG;
          break;
        }

      getDeriv(); // derivative (sets global variable)
      steerI += steerKI * err; // integral
      steerP = steerKP * err; // proportional

      doSteer(steerP, steerI, steerD); // Do actual steer
      break;

    } 
    case TAG: {
      lowerArm();
      turnAround();
      currState = TRACK;
      /* make it go back to home base? */
      /*forwardCam.ccc.getBlocks(32);*/
      break;
    }

  }
  // program to lift the arm using photoDiode
  // detect 
  delay(5);
}
