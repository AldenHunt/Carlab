#include <Pixy2.h>
#include <Servo.h>

Pixy2 forwardCam;
Servo mitt;
Servo steer;

int photoDiode = 12;
int objX = 168;
int err = 0;
int servoPos = 90;
int loops = 0;
int errorDerivative[5];

// PID variables for steering //
double steerP, steerI, steerD;
double steerKP, steerKI, steerKD; 

enum state {
  TRACK,
  FIELD,
  RUN,
  TAG
};


void setup() {
  // put your setup code here, to run once:
  // Attaching motors/servos/sensors to correct pin
  pinMode(photoDiode, INPUT);
  mitt.attach(9);
  steer.attach(10);

  steerKP = 2; steerKI = 0.2; steerKD = 1;
}

void doSteer(double P, double I, double D) {
  double toServo = 90 + P + I + D;
  if (toServo < 0) toServo = 0;
  else if (toServo > 180) toServo = 180;
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
  mitt.write(135);
  Serial.print("Lifted");
}

void lowerArm() {
  mitt.write(90);
  Serial.print("Lowered");
}

void loop() {
<<<<<<< HEAD
  // put your main code here, to run repeatedly:
  
  // Get the error for PID (based on x-location of ball in frame)
  forwardCam.ccc.getBlocks();
  // do we wannt to use forwardCam.ccc.getBlocks(true, 1) <-- this lets us
    //only look at the first signature and wait until it finds one witbout returning false
  for (int i = 0; i < (sizeof(forwardCam.ccc.blocks) / sizeof(forwardCam.ccc.blocks[0])); i++) {
    if (forwardCam.ccc.blocks[i].m_width < 5) continue; // Avoid false positives
    objX = forwardCam.ccc.blocks[i].m_x;
    err = objX - 168;
    break;
  }
=======
>>>>>>> c717cb57ebf79dff253b79a87f1010807aa02aab

  switch(state) {
    case TRACK: {

      if (photoDiode) {
        state = FIELD;
        break;
      }
      // Get the error for PID (based on x-location of ball in frame)
      forwardCam.ccc.getBlocks();

      /* ! THIS SHOULD PROBABLY BE A WHILE LOOP ! */
      for (int i = 0; i < (sizeof(forwardCam.ccc.blocks) / sizeof(forwardCam.ccc.blocks[0])); i++) {
        if (forwardCam.ccc.blocks[i].m_width < 5) continue; // Avoid false positives
        objX = forwardCam.ccc.blocks[i].m_x;
        err = objX - 168;
        break;
      }
    
      // Calculations for PID components
      getDeriv(); // derivative (sets global variable)
      loops++; 
      steerI += steerKI * err; // integral
      steerP = steerKP * err; // proportional

      doSteer(SteerP, SteerI, SteerD); // Do actual steer
      break;
    }
    case FIELD: {
      liftArm();
      state = RUN;
      break;
    }
  }
  
  
  
  



  // program to lift the arm using photoDiode
  // detect 

}
