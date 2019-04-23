#include <Pixy2.h>
#include <Servo.h>

Pixy2 forwardCam;
int photoDiode = 12;
Servo mitt;
Servo steer;
int objX = 168;
int err = 0;
int servoPos = 90;
 // variable 
void setup() {
  // put your setup code here, to run once:
  pinMode(photoDiode, INPUT);
  mitt.attach(9);
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
  // put your main code here, to run repeatedly:
  
  // Get the error for PID (based on x-location of ball in frame)
  /* forwardCam.ccc.getBlocks();
  for (int i = 0; i < (sizeof(forwardCam.ccc.blocks) / sizeof(forwardCam.ccc.blocks[0])); i++) {
    if (forwardCam.ccc.blocks[i].m_width < 5) continue; // Avoid false positives
    objX = forwardCam.ccc.blocks[i].m_x;
    err = objX - 168;
    break;
  } */

  liftArm();
  delay(500);
  lowerArm();
  delay(500);

}
