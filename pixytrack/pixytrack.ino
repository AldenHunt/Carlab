#include <Pixy2.h>
#include <Servo.h>

Pixy2 forwardCam;
Servo mitt;
Servo steer;

enum state {
  TRACK,
  FIELD,
  RUN,
  TAG
};

int photoDiode = 12;
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
  mitt.attach(9);
  steer.attach(10);
  Serial.begin(9600);
  forwardCam.init();

  loops = 0;
  currState = TRACK;
  steerKP = 0.5; steerKI = 0.0; steerKD = 0.0;
}

void doSteer(double P, double I, double D) {
  int toServo = (int) 90 + P + I + D;
  if (toServo < 0) toServo = 0;
  else if (toServo > 180) toServo = 180;
  Serial.print("Steer: ");
  Serial.println(toServo);
  steer.write(toServo);
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
<<<<<<< HEAD
  // put your main code here, to run repeatedly:
  Blocks ball[]; // ball block
  Blocks bases[]; // bases block

  // Get the error for PID (based on x-location of ball in frame)
  ball = forwardCam.ccc.getBlocks(true, 1); // wait until it detects object with signature 1 (ball) 
  for (int i = 0; i < (sizeof(forwardCam.ccc.blocks) / sizeof(forwardCam.ccc.blocks[0])); i++) {
    if (forwardCam.ccc.blocks[i].m_width < 5) continue; // Avoid false positives
    objX = forwardCam.ccc.blocks[i].m_x;
    err = objX - 168;
    break;
  }

  switch(state) {
=======
  switch(currState) {
>>>>>>> 1b83811fa50c0153e2a6ea7d08012ee3149bf074
    case TRACK: {

      /*if (photoDiode) {
        currState = FIELD;
        break;
      } */
      // Get the error for PID (based on x-location of ball in frame)
<<<<<<< HEAD
      ball = forwardCam.ccc.getBlocks(true, 1);
=======
      forwardCam.ccc.getBlocks();
      Serial.print("Blocks: ");
      Serial.println(forwardCam.ccc.numBlocks);
      //forwardCam.ccc.blocks[0].print();
      objX = forwardCam.ccc.blocks[0].m_x;
      
      err = objX - 168;
>>>>>>> 1b83811fa50c0153e2a6ea7d08012ee3149bf074

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
      liftArm();
      currState = RUN;
      break;
    }
    case RUN: {
      bases = forwardCam.ccc.getBlocks(true, 14); // we want signatures 1110, just the bases
      


    }
    case TAG: {
      lowerArm();
      
      break;
    }

    }
    }
  }
  
  
  
  delay(1000);



  // program to lift the arm using photoDiode
  // detect 

}
