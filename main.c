/*
    ( M2 ) ***    1^|     0^|   ****  ( M1 )

    ( M3 ) ***    1^|     0^|    **** ( M4 )
  146counts = 33cm,442counts=100cm
*/
#include "Wire.h"
//#include<MPU6050_tockn.h>
#include<Servo.h>
#include <SharpIR.h>

//MPU6050 mpu6050(Wire);
Servo Myservo1;
Servo Myservo2;

SharpIR sensor( SharpIR::GP2Y0A21YK0F, A9 );
int x = 0, tar, i, angletar, imu_input_prev, imu_input, e1;
int DIR_1 = 23, DIR_2 = 25, DIR_3 = 27, DIR_4 = 29, DIR_5B = 31, DIR_5F = 33, trig3 = A12;;
int PWM_1 = 8, PWM_2 = 9, PWM_3 = 10 , PWM_4 = 11, PWM_5 = 12, setPWM = 55;
int M1_ENCA = 19, M2_ENCA = 18, M3_ENCA = 2, M4_ENCA = 3;
int M1_ENCB = 7, M2_ENCB = 6, M3_ENCB = 5, M4_ENCB = 4, M5_ENCB = 51;
int a, pos_1 = 0, b, pos_2 = 0, c, pos_3 = 0, d, pos_4 = 0, e, pos_5 = 0;
int MPU;
long prevT = 0;
float eprev1 = 0, eprev2 = 0, eprev3 = 0, eprev4 = 0, eprev5 = 0;
float eIntegral_1 = 0, eIntegral_2 = 0, eIntegral_3 = 0, eIntegral_4 = 0, eIntegral_5 = 0;
/////////////////////ultrasonics//////////////////////
unsigned long counter_1, counter_2, counter_3, current_time, distance1;
byte last_ech1_state, last_ech2_state, last_ech3_state;
void pidForward();
void pidBackward();
void MpuPID();
void updownPID();
///////////////////CASESS///////////////////////
typedef enum Bot {INI, FORWARD1,RIGHTWARD1, RIGHTWARD2,RIGHTWARD3,RIGHTWARD4, ROTATION1, ROTATION2,ROTATION3,ROTATION4, FORWARD2, UPDOWN1, UPDOWN2, SHARP1, SHARP2, IMU1, SERVO1, FORWARD3, FORWARD4,FORWARD5,FORWARD6, SERVO2, SERVO3,SERVO4,SERVO5, BACKWARD1, BACKWARD2,BACKWARD3,BACKWARD4, LEFTWARD1, LEFTWARD2};
unsigned int Bot = INI;

void setup() {
  Serial.begin(2000000);
//  Wire.begin();
//  mpu6050.begin();
//  mpu6050.calcGyroOffsets(true);
  PCICR |= (1 << PCIE0); // enc5
  PCMSK0 |= (1 << PCINT0);
  pinMode(A0, INPUT);
  pinMode(A14, OUTPUT);
  pinMode(A15, OUTPUT);
  pinMode(DIR_1, OUTPUT);  pinMode(DIR_2, OUTPUT);
  pinMode(DIR_3, OUTPUT);  pinMode(DIR_4, OUTPUT);
  pinMode(DIR_5F, OUTPUT); pinMode(DIR_5B, OUTPUT);
  pinMode(PWM_1, OUTPUT);  pinMode(PWM_2, OUTPUT);
  pinMode(PWM_3, OUTPUT);  pinMode(PWM_4, OUTPUT);  pinMode(PWM_5, OUTPUT);
  pinMode(M1_ENCA, INPUT); pinMode(M1_ENCB, INPUT);
  pinMode(M2_ENCA, INPUT); pinMode(M2_ENCB, INPUT);
  pinMode(M3_ENCA, INPUT); pinMode(M3_ENCB, INPUT);
  pinMode(M4_ENCA, INPUT); pinMode(M4_ENCB, INPUT); pinMode(M5_ENCB, INPUT);
  attachInterrupt(digitalPinToInterrupt(M1_ENCA), M1update_encoder, RISING);
  attachInterrupt(digitalPinToInterrupt(M2_ENCA), M2update_encoder, RISING);
  attachInterrupt(digitalPinToInterrupt(M3_ENCA), M3update_encoder, RISING);
  attachInterrupt(digitalPinToInterrupt(M4_ENCA), M4update_encoder, RISING);
  int Se = Myservo1.attach(46);
  int Sr = Myservo2.attach(44);

}
/////////////////////////////////////////////////////////////////////////////////LOOP//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
 // mpu6050.update();
  switch (Bot) {
    case INI:
      x++;
      if (x == 2) {
        Serial.println("INT enddddd");
        exit(0);
      }
      Serial.println("INT PASS");
      Bot =  SERVO1; // SERVO1 FORWARD1  UPDOWN1  UPDOWN3 LEFTWARD1 BACKWARD1
      break;
 ////////////////////////////SERVO1/////////////////////////////////////
    case SERVO1:
    analogWrite(A15, 255);
      Serial.print("Servo1 enter ");
      Myservo1.write(0);
      Myservo2.write(0);
      delay(1000);
      Bot = FORWARD1;
      break;
    ////////////////////////////FORWARD1/////////////////////////////////////
    case FORWARD1:
      tar = -212;
      PidForward(tar);
      Serial.print("M1Count: ");
      Serial.print(pos_1);
      Serial.print("  M2Count: ");
      Serial.print(pos_2);
      Serial.print("  M3Count: ");
      Serial.print(pos_3);
      Serial.print("  M4Count: ");
      Serial.println(pos_4);
      if ((pos_1 <= -211 && pos_1 >= -213) && (pos_2 >= 211 && pos_2 <= 213) && (pos_3 >= 211 && pos_3 <= 213) && (pos_4 <= -211 && pos_4 >= -213)) {
        pos_1 = 0; pos_2 = 0; pos_3 = 0; pos_4 = 0;
        analogWrite(PWM_1, 0); analogWrite(PWM_2, 0); analogWrite(PWM_3, 0); analogWrite(PWM_4, 0);
        delay(1000);
        Bot = RIGHTWARD1;
      }
      else Bot = FORWARD1;
      break;
    ////////////////////////////RIGHTWARD1:///////////////////////////////////
    case RIGHTWARD1:
      tar = 203;
      PidRightward(tar);
      Serial.print("M1Count: ");
      Serial.print(pos_1);
      Serial.print("  M2Count: ");
      Serial.print(pos_2);
      Serial.print("  M3Count: ");
      Serial.print(pos_3);
      Serial.print("  M4Count: ");
      Serial.println(pos_4);
      if ((pos_1 >= 202 && pos_1 <= 204) && (pos_2 >= 202 && pos_2 <= 204) && (pos_3 <= -202 && pos_3 >= -204) && (pos_4 <= -202 && pos_4 >= -204)) {
        pos_1 = 0; pos_2 = 0; pos_3 = 0; pos_4 = 0; analogWrite(PWM_1, 0); analogWrite(PWM_2, 0); analogWrite(PWM_3, 0); analogWrite(PWM_4, 0);
        delay(1000);
        Bot = ROTATION1;
      }
      else{ Bot = RIGHTWARD1;}
      break;
    ////////////////////////////ROTATION1:///////////////////////////////////
    case ROTATION1:
      tar = -328;
      PidRotation(tar);
      Serial.print("M1Count: ");
      Serial.print(pos_1);
      Serial.print("  M2Count: ");
      Serial.print(pos_2);
      Serial.print("  M3Count: ");
      Serial.print(pos_3);
      Serial.print("  M4Count: ");
      Serial.println(pos_4);
      if ((pos_1 <= -327 && pos_1 >= -329) && (pos_2 <= -327&& pos_2 >= -329) && (pos_3 <= -327 && pos_3 >= -329) && (pos_4 <= -327&& pos_4 >= -329)) {
        pos_1 = 0; pos_2 = 0; pos_3 = 0; pos_4 = 0; 
        analogWrite(PWM_1, 0); analogWrite(PWM_2, 0); analogWrite(PWM_3, 0); analogWrite(PWM_4, 0);
        delay(1000);
        Bot = FORWARD2 ;
      }
      else {Bot = ROTATION1;}
      break;
  ////////////////////////////FORWARD2/////////////////////////////////////
    case FORWARD2:
      tar = -130;
      PidForward(tar);
      Serial.print("M1Count: ");
      Serial.print(pos_1);
      Serial.print("  M2Count: ");
      Serial.print(pos_2);
      Serial.print("  M3Count: ");
      Serial.print(pos_3);
      Serial.print("  M4Count: ");
      Serial.println(pos_4);
      if ((pos_1 <= -129 && pos_1 >= -131) && (pos_2 >= 129&& pos_2 <= 131) && (pos_3 >= 129&& pos_3 <= 131) && (pos_4 <=129&& pos_4 >= -131)) {
        pos_1 = 0; pos_2 = 0; pos_3 = 0; pos_4 = 0; analogWrite(PWM_1, 0); analogWrite(PWM_2, 0); analogWrite(PWM_3, 0); analogWrite(PWM_4, 0);
        delay(1000);
        Bot = UPDOWN1;
      }
      else { Bot = FORWARD2; }
      break;
  ////////////////////////////////UPDOWN1://///////////////////////////////
    case UPDOWN1:
      Serial.println("UPDOWN1 ENTR:");
      tar = 900;
      Serial.println(pos_5);
      updownPID(tar);
      if (pos_5 >= 899 && pos_5 <= 901) {
        digitalWrite(DIR_5F, 1);
        digitalWrite(DIR_5B, 0);
        analogWrite(PWM_5, 60);
        pos_1 = 0; pos_2 = 0; pos_3 = 0; pos_4 = 0;
        delay(1000);
        Bot = SHARP1; // SHARP1 INI  UPDOWN2
       }
      else { Bot = UPDOWN1; }
      break;
 //////////////////////////////sharp1://///////////////////////////////
    case SHARP1:
      Serial.print("Sharp1 enter ");
      tar = 22;
      sharp1PID(tar);
      Serial.println(distance1);
      if (distance1 == 22) {
        pos_1 = 0; pos_2 = 0; pos_3 = 0; pos_4 = 0;
        delay(1000);
        Bot = FORWARD3;
      }
      else { Bot = SHARP1; }
      break;

 ////////////////////////////FORWARD3/////////////////////////////////////
    case FORWARD3:
      tar = -50;
      PidForward(tar);
      Serial.print("M1Count: ");
      Serial.print(pos_1);
      Serial.print("  M2Count: ");
      Serial.print(pos_2);
      Serial.print("  M3Count: ");
      Serial.print(pos_3);
      Serial.print("  M4Count: ");
      Serial.println(pos_4);
      if ((pos_1 <= -49 && pos_1 >= -51) && (pos_2 >= 49&& pos_2 <= 51) && (pos_3 >= 49&& pos_3 <= 51) && (pos_4 <=49&& pos_4 >= -51)) {
        pos_1 = 0; pos_2 = 0; pos_3 = 0; pos_4 = 0; analogWrite(PWM_1, 0); analogWrite(PWM_2, 0); analogWrite(PWM_3, 0); analogWrite(PWM_4, 0);
        delay(1000);
        Bot = SERVO2;
      }
      else { Bot = FORWARD3;}
      break;
 ////////////////////////////SERVO2/////////////////////////////////////
    case SERVO2:
      setPWM = 55;
      Myservo1.write(70);
      delay(1000);
      Bot = BACKWARD1;
      break;
 //////////////////////////////BACKWARD://///////////////////////////////
    case BACKWARD1:
      tar = 265;
      PidBackward(tar);
      Serial.print("M1Count: ");
      Serial.print(pos_1);
      Serial.print("  M2Count: ");
      Serial.print(pos_2);
      Serial.print("  M3Count: ");
      Serial.print(pos_3);
      Serial.print("  M4Count: ");
      Serial.println(pos_4);
      if ((pos_1 >= 265& pos_1 <= 266) && (pos_2 <= -265 && pos_2 >= -266) && (pos_3 <= -265&& pos_3 >= -266) && (pos_4 >= 265&& pos_4 <= 266)) {
        pos_1 = 0; pos_2 = 0; pos_3 = 0; pos_4 = 0; analogWrite(PWM_1, 0); analogWrite(PWM_2, 0); analogWrite(PWM_3, 0); analogWrite(PWM_4, 0);
        delay(1000);
        Bot = ROTATION2;
      }
      else {Bot = BACKWARD1;}
      break;
 //////////////////////////////ROTATION2:////////////////////////////////
    case ROTATION2:
      tar = -328;    //360deg
      PidRotation(tar);
      Serial.print("M1Count: ");
      Serial.print(pos_1);
      Serial.print("  M2Count: ");
      Serial.print(pos_2);
      Serial.print("  M3Count: ");
      Serial.print(pos_3);
      Serial.print("  M4Count: ");
      Serial.println(pos_4);
      if ((pos_1 <= -327 && pos_1 >= -329) && (pos_2 <= -327&& pos_2 >= -329) && (pos_3 <= -327 && pos_3 >= -329) && (pos_4 <= -327&& pos_4 >= -329)) {
        pos_1 = 0; pos_2 = 0; pos_3 = 0; pos_4 = 0; analogWrite(PWM_1, 0); analogWrite(PWM_2, 0); analogWrite(PWM_3, 0); analogWrite(PWM_4, 0);
        delay(1000);
        Bot = RIGHTWARD2 ;
      }
      else {Bot = ROTATION2; }
      break;
 ////////////////////////////RIGHTWARD2:///////////////////////////////////
    case RIGHTWARD2:
      tar = 470;      // +49cm
      PidRightward(tar);
      Serial.print("M1Count: ");
      Serial.print(pos_1);
      Serial.print("  M2Count: ");
      Serial.print(pos_2);
      Serial.print("  M3Count: ");
      Serial.print(pos_3);
      Serial.print("  M4Count: ");
      Serial.println(pos_4);
      if ((pos_1 >= 467 && pos_1 <= 471) && (pos_2 >= 467 && pos_2 <= 471) && (pos_3 <= -467 && pos_3 >= -471) && (pos_4 <= -467 && pos_4 >= -471)) {
        pos_1 = 0; pos_2 = 0; pos_3 = 0; pos_4 = 0; analogWrite(PWM_1, 0); analogWrite(PWM_2, 0); analogWrite(PWM_3, 0); analogWrite(PWM_4, 0);
        delay(1000);
        Bot = UPDOWN2;
      }
      else Bot = RIGHTWARD2;
      break;
 ////////////////////////////////UPDOWN2://///////////////////////////////
  case UPDOWN2:
      Serial.println("UPDOWN2 ENTR:");
      tar = 65;
      Serial.println(pos_5);
      updownPID(tar);
      if (pos_5 >= 64 && pos_5 <= 66) {
        digitalWrite(DIR_5F, 1);
        digitalWrite(DIR_5B, 0);
        analogWrite(PWM_5, 100);
        delay(1300);
        Bot = FORWARD4; //FORWARD4 INI
      }
      else { Bot = UPDOWN2; }
      break;
 ////////////////////////////FORWARD4/////////////////////////////////////
    case FORWARD4:
      tar = -350;    //52cm
      PidForward(tar);
      Serial.print("M1Count: ");
      Serial.print(pos_1);
      Serial.print("  M2Count: ");
      Serial.print(pos_2);
      Serial.print("  M3Count: ");
      Serial.print(pos_3);
      Serial.print("  M4Count: ");
      Serial.println(pos_4);
      if ((pos_1 <= -348 && pos_1 >= -351) && (pos_2 >= 348&& pos_2 <= 351) && (pos_3 >= 348&& pos_3 <= 351) && (pos_4 <= -348&& pos_4 >= -351)) {
        pos_1 = 0; pos_2 = 0; pos_3 = 0; pos_4 = 0;
        analogWrite(PWM_1, 0); analogWrite(PWM_2, 0); analogWrite(PWM_3, 0); analogWrite(PWM_4, 0);
        delay(1000);
        Bot = SHARP2;
      }
      else {
        Bot = FORWARD4;
      }
      break;
 //////////////////////////////sharp2://///////////////////////////////
    case SHARP2:
      setPWM = 55;
      tar = 19;
      sharp1PID(tar);
      Serial.println(distance1);
      if (distance1 == 19) {
        pos_1 = 0; pos_2 = 0; pos_3 = 0; pos_4 = 0;
        delay(1000);
        Bot = SERVO3;
      }
      else {
        Bot = SHARP2;
      }
      break;

 ////////////////////////////SERVO3/////////////////////////////////////
    case SERVO3:
      Serial.println("serv3 ENTR:");
      Myservo1.write(0);
      Myservo2.write(0);
      pos_1 = 0; pos_2 = 0; pos_3 = 0; pos_4 = 0;
      delay(1000);
      Bot = BACKWARD3;
      break;
 /////////////////////////////////////////////////////////////////////////
 //////////////////////////////BACKWARD2://///////////////////////////////
    case BACKWARD3:
      tar = 347;
      PidBackward(tar);
      Serial.print("M1Count: ");
      Serial.print(pos_1);
      Serial.print("  M2Count: ");
      Serial.print(pos_2);
      Serial.print("  M3Count: ");
      Serial.print(pos_3);
      Serial.print("  M4Count: ");
      Serial.println(pos_4);
      if ((pos_1 >= 346 & pos_1 <= 348) && (pos_2 <= -346&& pos_2 >= -348) && (pos_3 <= -346&& pos_3 >= -348) && (pos_4 >= 346&& pos_4 <= 348)) {
        pos_1 = 0; pos_2 = 0; pos_3 = 0; pos_4 = 0; analogWrite(PWM_1, 0); analogWrite(PWM_2, 0); analogWrite(PWM_3, 0); analogWrite(PWM_4, 0);
        delay(1000);
        Bot = RIGHTWARD4;
      }
      else {Bot = BACKWARD3;}
      break;    
////////////////////////////RIGHTWARD4:///////////////////////////////////
    case RIGHTWARD4:
      tar = -680;
      PidRightward(tar);
      Serial.print("M1Count: ");
      Serial.print(pos_1);
      Serial.print("  M2Count: ");
      Serial.print(pos_2);
      Serial.print("  M3Count: ");
      Serial.print(pos_3);
      Serial.print("  M4Count: ");
      Serial.println(pos_4);
      if ((pos_1<= -678 && pos_1>= -685) && (pos_2 <= -678&& pos_2 >= -685) && (pos_3 >= 678&& pos_3 <= 685) && (pos_4 >= 678&& pos_4 <= 685)) {
        pos_1 = 0; pos_2 = 0; pos_3 = 0; pos_4 = 0; analogWrite(PWM_1, 0); analogWrite(PWM_2, 0); analogWrite(PWM_3, 0); analogWrite(PWM_4, 0);
        delay(1000);
        Bot = BACKWARD4;
      }
      else Bot = RIGHTWARD4;
      break;   
//////////////////////////////BACKWARD4://///////////////////////////////
    case BACKWARD4:
     tar = 347;
      PidBackward(tar);
      Serial.print("M1Count: ");
      Serial.print(pos_1);
      Serial.print("  M2Count: ");
      Serial.print(pos_2);
      Serial.print("  M3Count: ");
      Serial.print(pos_3);
      Serial.print("  M4Count: ");
      Serial.println(pos_4);
      if ((pos_1 >= 345&& pos_1 <= 348) && (pos_2 <= -345&& pos_2 >= -348) && (pos_3 <= -345&& pos_3 >= -348) && (pos_4 >= 345&& pos_4 <= 348)) {
        analogWrite(A14,255);
        analogWrite(A15, 0);
        pos_1 = 0; pos_2 = 0; pos_3 = 0; pos_4 = 0; analogWrite(PWM_1, 0); analogWrite(PWM_2, 0); analogWrite(PWM_3, 0); analogWrite(PWM_4, 0);
        delay(10000);
        exit(0);
      }
      else {Bot = BACKWARD4;}
      break;             
      default:
      Serial.println("............");
  }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////sharpIR///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void sharp1PID(int tar) {
  distance1 = sensor.getDistance();
  int target1 = tar;
  float kp = 17, ki = 14, kd = 0;
  long currT = micros();
  float deltaT = ((float)(currT - prevT)) / 1.0e6;
  prevT = currT;
  int e1 = distance1 - target1;
  float dedt_1 = (e1 - eprev1) / deltaT;
  float eIntegral_1 = eIntegral_1 + e1 * deltaT;
  float u1 = kp * e1 + kd * dedt_1 + ki * eIntegral_1;
  float u2 = kp * e1 + kd * dedt_1 + ki * eIntegral_1;
  float u3 = kp * e1 + kd * dedt_1 + ki * eIntegral_1;
  float u4 = kp * e1 + kd * dedt_1 + ki * eIntegral_1;
  float pwr1 = fabs(u1), pwr2 = fabs(u2), pwr3 = fabs(u3), pwr4 = fabs(u4);
  if (pwr1 > setPWM) {
    pwr1 = setPWM;
  }
  if (pwr2 > setPWM) {
    pwr2 = setPWM;
  }
  if (pwr3 > setPWM) {
    pwr3 = setPWM;
  }
  if (pwr4 > setPWM) {
    pwr4 = setPWM;
  }
  int dir1 = -1, dir2 = 1 , dir3 = -1, dir4 = 1 ;
  if (u1 < 0 ) {
    dir1 = 1;
  }
  if (u2 < 0)  {
    dir2 = -1;
  }
  if (u3 < 0 ) {
    dir3 = 1;
  }
  if (u4 < 0)  {
    dir4 = -1;
  }
  setMotor1(dir1, pwr1, PWM_1, M1_ENCA, M1_ENCB);
  setMotor2(dir2, pwr2, PWM_2, M2_ENCA, M2_ENCB);
  setMotor3(dir3, pwr3, PWM_3, M3_ENCA, M3_ENCB);
  setMotor4(dir4, pwr4, PWM_4, M4_ENCA, M4_ENCB);
  eprev1 = e1;
}

////////////////////////////////////////////////////updown///////////////////////////////////////////////////////////
void updownPID(int tar) {
  int target5 = tar;
  float kp = 17, ki = 14, kd = 1;
  long currT = micros();
  float deltaT = ((float)(currT - prevT)) / 1.0e6;
  prevT = currT;
  int e5 = pos_5 - target5;
  ////derivative/////
  float dedt_5 = (e5 - eprev5) / deltaT;
  /////Integral//////
  float eIntegral_5 = eIntegral_5 + e5 * deltaT;
  ////////control signal//////
  float u5 = kp * e5 + kd * dedt_5 + ki * eIntegral_5;
  float pwr5 = fabs(u5);
  if (pwr5 > 240) {  pwr5 = 240; }
  int dir5 = -1;
  if (u5 < 0 ) {
    dir5 = 1;
  }
  setMotor5(dir5 , pwr5, PWM_5, M5_ENCB);
  eprev5 = e5;
}

//////////////////////////////////////////////ForWard////////////////////////////////////////////////////////////
void PidForward(int tar) {
  int target1 = tar, target2 = -tar, target3 = -tar, target4 = tar;
  float kp = 17, ki = 14, kd = 0;
  long currT = micros();
  float deltaT = ((float)(currT - prevT)) / 1.0e6;
  prevT = currT;
  int e1 = pos_1 - target1;
  int e2 = pos_2 - target2;
  int e3 = pos_3 - target3;
  int e4 = pos_4 - target4;
  ////derivative/////
  float dedt_1 = (e1 - eprev1) / deltaT;
  float dedt_2 = (e2 - eprev2) / deltaT;
  float dedt_3 = (e3 - eprev3) / deltaT;
  float dedt_4 = (e4 - eprev4) / deltaT;
  /////Integral//////
  float eIntegral_1 = eIntegral_1 + e1 * deltaT;
  float eIntegral_2 = eIntegral_2 + e2 * deltaT;
  float eIntegral_3 = eIntegral_3 + e3 * deltaT;
  float eIntegral_4 = eIntegral_4 + e4 * deltaT;
  ////////control signal//////
  float u1 = kp * e1 + kd * dedt_1 + ki * eIntegral_1;
  float u2 = kp * e2 + kd * dedt_2 + ki * eIntegral_2;
  float u3 = kp * e3 + kd * dedt_3 + ki * eIntegral_3;
  float u4 = kp * e4 + kd * dedt_4 + ki * eIntegral_4;
  float pwr1 = fabs(u1), pwr2 = fabs(u2), pwr3 = fabs(u3), pwr4 = fabs(u4);
  if (pwr1 > setPWM) {
    pwr1 = setPWM;
  }
  if (pwr2 > setPWM) {
    pwr2 = setPWM;
  }
  if (pwr3 > setPWM) {
    pwr3 = setPWM;
  }
  if (pwr4 > setPWM) {
    pwr4 = setPWM;
  }
  int dir1 = 1, dir2 = -1 , dir3 = -1, dir4 = 1 ;
  if (u1 < 0 ) {
    dir1 = -1;
  }
  if (u2 < 0)  {
    dir2 = 1;
  }
  if (u3 < 0 ) {
    dir3 = 1;
  }
  if (u4 < 0)  {
    dir4 = -1;
  }
  setMotor1(dir1, pwr1, PWM_1, M1_ENCA, M1_ENCB);
  setMotor2(dir2, pwr2, PWM_2, M2_ENCA, M2_ENCB);
  setMotor3(dir3, pwr3, PWM_3, M3_ENCA, M3_ENCB);
  setMotor4(dir4, pwr4, PWM_4, M4_ENCA, M4_ENCB);
  eprev1 = e1; eprev2 = e2; eprev3 = e3; eprev4 = e4;
}

///////////////////////////////////////////BackWard//////////////////////////////////////////////////////////////

void PidBackward(int tar) {
  int target1 = tar, target2 = -tar, target3 = -tar, target4 = tar;
  float kp = 17, ki = 14, kd = 0;
  long currT = micros();
  float deltaT = ((float)(currT - prevT)) / 1.0e6;
  prevT = currT;
  int e1 = pos_1 - target1;
  int e2 = pos_2 - target2;
  int e3 = pos_3 - target3;
  int e4 = pos_4 - target4;
  ////derivative/////
  float dedt_1 = (e1 - eprev1) / deltaT;
  float dedt_2 = (e2 - eprev2) / deltaT;
  float dedt_3 = (e3 - eprev3) / deltaT;
  float dedt_4 = (e4 - eprev4) / deltaT;
  /////Integral//////
  float eIntegral_1 = eIntegral_1 + e1 * deltaT;
  float eIntegral_2 = eIntegral_2 + e2 * deltaT;
  float eIntegral_3 = eIntegral_3 + e3 * deltaT;
  float eIntegral_4 = eIntegral_4 + e4 * deltaT;
  ////////control signal//////
  float u1 = kp * e1 + kd * dedt_1 + ki * eIntegral_1;
  float u2 = kp * e2 + kd * dedt_2 + ki * eIntegral_2;
  float u3 = kp * e3 + kd * dedt_3 + ki * eIntegral_3;
  float u4 = kp * e4 + kd * dedt_4 + ki * eIntegral_4;
  float pwr1 = fabs(u1), pwr2 = fabs(u2), pwr3 = fabs(u3), pwr4 = fabs(u4);
  if (pwr1 > setPWM) {
    pwr1 = setPWM;
  }
  if (pwr2 > setPWM) {
    pwr2 = setPWM;
  }
  if (pwr3 > setPWM) {
    pwr3 = setPWM;
  }
  if (pwr4 > setPWM) {
    pwr4 = setPWM;
  }
  int dir1 = 1, dir2 = -1 , dir3 = -1, dir4 = 1 ;
  if (u1 < 0 ) {
    dir1 = -1;
  }
  if (u2 < 0)  {
    dir2 = 1;
  }
  if (u3 < 0 ) {
    dir3 = 1;
  }
  if (u4 < 0)  {
    dir4 = -1;
  }
  setMotor1(dir1, pwr1, PWM_1, M1_ENCA, M1_ENCB);
  setMotor2(dir2, pwr2, PWM_2, M2_ENCA, M2_ENCB);
  setMotor3(dir3, pwr3, PWM_3, M3_ENCA, M3_ENCB);
  setMotor4(dir4, pwr4, PWM_4, M4_ENCA, M4_ENCB);
  eprev1 = e1; eprev2 = e2; eprev3 = e3; eprev4 = e4;
}


///////////////////////////////////////////RightWard//////////////////////////////////////////////////////////////////

void PidRightward(int tar) {
  int target1 = tar, target2 = tar, target3 = -tar, target4 = -tar;
  float kp = 17, ki = 14, kd = 0;
  long currT = micros();
  float deltaT = ((float)(currT - prevT)) / 1.0e6;
  prevT = currT;
  int e1 = pos_1 - target1;
  int e2 = pos_2 - target2;
  int e3 = pos_3 - target3;
  int e4 = pos_4 - target4;
  ////derivative/////
  float dedt_1 = (e1 - eprev1) / deltaT;
  float dedt_2 = (e2 - eprev2) / deltaT;
  float dedt_3 = (e3 - eprev3) / deltaT;
  float dedt_4 = (e4 - eprev4) / deltaT;
  /////Integral//////
  float eIntegral_1 = eIntegral_1 + e1 * deltaT;
  float eIntegral_2 = eIntegral_2 + e2 * deltaT;
  float eIntegral_3 = eIntegral_3 + e3 * deltaT;
  float eIntegral_4 = eIntegral_4 + e4 * deltaT;
  ////////control signal//////
  float u1 = kp * e1 + kd * dedt_1 + ki * eIntegral_1;
  float u2 = kp * e2 + kd * dedt_2 + ki * eIntegral_2;
  float u3 = kp * e3 + kd * dedt_3 + ki * eIntegral_3;
  float u4 = kp * e4 + kd * dedt_4 + ki * eIntegral_4;
  float pwr1 = fabs(u1), pwr2 = fabs(u2), pwr3 = fabs(u3), pwr4 = fabs(u4);
  if (pwr1 > setPWM) { pwr1 = setPWM; }
  if (pwr2 > setPWM) { pwr2 = setPWM;}
  if (pwr3 > setPWM) { pwr3 = setPWM;}
  if (pwr4 > setPWM) { pwr4 = setPWM;}
  int dir1 = 1, dir2 = -1 , dir3 = -1, dir4 = 1 ;
  if (u1 < 0 ) {
    dir1 = -1;
  }
  if (u2 < 0)  {
    dir2 = 1;
  }
  if (u3 < 0 ) {
    dir3 = 1;
  }
  if (u4 < 0)  {
    dir4 = -1;
  }
  setMotor1(dir1, pwr1, PWM_1, M1_ENCA, M1_ENCB);
  setMotor2(dir2, pwr2, PWM_2, M2_ENCA, M2_ENCB);
  setMotor3(dir3, pwr3, PWM_3, M3_ENCA, M3_ENCB);
  setMotor4(dir4, pwr4, PWM_4, M4_ENCA, M4_ENCB);
  eprev1 = e1; eprev2 = e2; eprev3 = e3; eprev4 = e4;
}

///////////////////////////////////////////LeftWard//////////////////////////////////////////////////////////////////

void PidLeftward(int tar) {
  int target1 = -tar, target2 = -tar, target3 = tar, target4 = tar;
  float kp = 17, ki = 14, kd = 0;
  long currT = micros();
  float deltaT = ((float)(currT - prevT)) / 1.0e6;
  prevT = currT;
  int e1 = pos_1 - target1;
  int e2 = pos_2 - target2;
  int e3 = pos_3 - target3;
  int e4 = pos_4 - target4;
  ////derivative/////
  float dedt_1 = (e1 - eprev1) / deltaT;
  float dedt_2 = (e2 - eprev2) / deltaT;
  float dedt_3 = (e3 - eprev3) / deltaT;
  float dedt_4 = (e4 - eprev4) / deltaT;
  /////Integral//////
  float eIntegral_1 = eIntegral_1 + e1 * deltaT;
  float eIntegral_2 = eIntegral_2 + e2 * deltaT;
  float eIntegral_3 = eIntegral_3 + e3 * deltaT;
  float eIntegral_4 = eIntegral_4 + e4 * deltaT;
  ////////control signal//////
  float u1 = kp * e1 + kd * dedt_1 + ki * eIntegral_1;
  float u2 = kp * e2 + kd * dedt_2 + ki * eIntegral_2;
  float u3 = kp * e3 + kd * dedt_3 + ki * eIntegral_3;
  float u4 = kp * e4 + kd * dedt_4 + ki * eIntegral_4;
  float pwr1 = fabs(u1), pwr2 = fabs(u2), pwr3 = fabs(u3), pwr4 = fabs(u4);
  if (pwr1 > setPWM) {
    pwr1 = setPWM;
  }
  if (pwr2 > setPWM) {
    pwr2 = setPWM;
  }
  if (pwr3 > setPWM) {
    pwr3 = setPWM;
  }
  if (pwr4 > setPWM) {
    pwr4 = setPWM;
  }
  int dir1 = 1, dir2 = -1 , dir3 = -1, dir4 = 1 ;
  if (u1 < 0 ) {
    dir1 = -1;
  }
  if (u2 < 0)  {
    dir2 = 1;
  }
  if (u3 < 0 ) {
    dir3 = 1;
  }
  if (u4 < 0)  {
    dir4 = -1;
  }
  setMotor1(dir1, pwr1, PWM_1, M1_ENCA, M1_ENCB);
  setMotor2(dir2, pwr2, PWM_2, M2_ENCA, M2_ENCB);
  setMotor3(dir3, pwr3, PWM_3, M3_ENCA, M3_ENCB);
  setMotor4(dir4, pwr4, PWM_4, M4_ENCA, M4_ENCB);
  eprev1 = e1; eprev2 = e2; eprev3 = e3; eprev4 = e4;
}

///////////////////////////////////////////ROTATION//////////////////////////////////////////////////////////////////

void PidRotation(int tar) {
  int target1 = tar, target2 = tar, target3 = tar, target4 = tar;
  float kp = 17, ki = 14, kd = 0;

  long currT = micros();
  float deltaT = ((float)(currT - prevT)) / 1.0e6;
  prevT = currT;

  int e1 = pos_1 - target1;
  int e2 = pos_2 - target2;
  int e3 = pos_3 - target3;
  int e4 = pos_4 - target4;

  ////derivative/////
  float dedt_1 = (e1 - eprev1) / deltaT;
  float dedt_2 = (e2 - eprev2) / deltaT;
  float dedt_3 = (e3 - eprev3) / deltaT;
  float dedt_4 = (e4 - eprev4) / deltaT;

  /////Integral//////
  float eIntegral_1 = eIntegral_1 + e1 * deltaT;
  float eIntegral_2 = eIntegral_2 + e2 * deltaT;
  float eIntegral_3 = eIntegral_3 + e3 * deltaT;
  float eIntegral_4 = eIntegral_4 + e4 * deltaT;

  ////////control signal//////
  float u1 = kp * e1 + kd * dedt_1 + ki * eIntegral_1;
  float u2 = kp * e2 + kd * dedt_2 + ki * eIntegral_2;
  float u3 = kp * e3 + kd * dedt_3 + ki * eIntegral_3;
  float u4 = kp * e4 + kd * dedt_4 + ki * eIntegral_4;

  float pwr1 = fabs(u1), pwr2 = fabs(u2), pwr3 = fabs(u3), pwr4 = fabs(u4);

  if (pwr1 > setPWM) {
    pwr1 = setPWM;
  }
  if (pwr2 > setPWM) {
    pwr2 = setPWM;
  }
  if (pwr3 > setPWM) {
    pwr3 = setPWM;
  }
  if (pwr4 > setPWM) {
    pwr4 = setPWM;
  }

  int dir1 = 1, dir2 = -1 , dir3 = -1, dir4 = 1 ;
  if (u1 < 0 ) {
    dir1 = -1;
  }
  if (u2 < 0)  {
    dir2 = 1;
  }
  if (u3 < 0 ) {
    dir3 = 1;
  }
  if (u4 < 0)  {
    dir4 = -1;
  }
  setMotor1(dir1, pwr1, PWM_1, M1_ENCA, M1_ENCB);
  setMotor2(dir2, pwr2, PWM_2, M2_ENCA, M2_ENCB);
  setMotor3(dir3, pwr3, PWM_3, M3_ENCA, M3_ENCB);
  setMotor4(dir4, pwr4, PWM_4, M4_ENCA, M4_ENCB);
  eprev1 = e1; eprev2 = e2; eprev3 = e3; eprev4 = e4;
}
///////////////////////////////////////////IMU//////////////////////////////////////////////////////////////////

void MpuPID(int tar) {
  //mpu6050.update();
  //MPU = mpu6050.getAngleZ();
  int target1 = tar;
  float kp = 17, ki = 14, kd = 0;
  long currT = micros();
  float deltaT = ((float)(currT - prevT)) / 1.0e6;
  prevT = currT;
  int e1 = MPU - target1;
  float dedt_1 = (e1 - eprev1) / deltaT;
  float eIntegral_1 = eIntegral_1 + e1 * deltaT;
  float u1 = kp * e1 + kd * dedt_1 + ki * eIntegral_1;
  float u2 = kp * e1 + kd * dedt_1 + ki * eIntegral_1;
  float u3 = kp * e1 + kd * dedt_1 + ki * eIntegral_1;
  float u4 = kp * e1 + kd * dedt_1 + ki * eIntegral_1;
  float pwr1 = fabs(u1), pwr2 = fabs(u2), pwr3 = fabs(u3), pwr4 = fabs(u4);
  if (pwr1 > setPWM) {
    pwr1 = setPWM;
  }
  if (pwr2 > setPWM) {
    pwr2 = setPWM;
  }
  if (pwr3 > setPWM) {
    pwr3 = setPWM;
  }
  if (pwr4 > setPWM) {
    pwr4 = setPWM;
  }
  int dir1 = -1, dir2 = 1 , dir3 = 1, dir4 = -1 ;
  if (u1 < 0 ) {
    dir1 = 1;
  }
  if (u2 < 0)  {
    dir2 = -1;
  }
  if (u3 < 0 ) {
    dir3 = -1;
  }
  if (u4 < 0)  {
    dir4 = 1;
  }
  setMotor1(dir1, pwr1, PWM_1, M1_ENCA, M1_ENCB);
  setMotor2(dir2, pwr2, PWM_2, M2_ENCA, M2_ENCB);
  setMotor3(dir3, pwr3, PWM_3, M3_ENCA, M3_ENCB);
  setMotor4(dir4, pwr4, PWM_4, M4_ENCA, M4_ENCB);
  eprev1 = e1;
}

//////////////////////////////////////////////////////////////////***************encoder directions*****************************//////////////////////////////////////////////////////////////////////////////////

void setMotor1(int dir1 , int pwmval1, int PWM1, int M1_ENCA, int M1_ENCB)
{
  analogWrite(PWM1, pwmval1);
  if (dir1 == 1) {
    digitalWrite(DIR_1, 1);
  }
  else if (dir1 == -1) { //LOW
    digitalWrite(DIR_1, 0);
  }
}

void setMotor2(int dir2 , int pwmval2, int PWM2, int M2_ENCA, int M2_ENCB)
{
  analogWrite(PWM2, pwmval2);
  if (dir2 == 1) {
    digitalWrite(DIR_2, 1);
  }
  else if (dir2 == -1) { //LOW
    digitalWrite(DIR_2, 0);
  }

}

void setMotor3(int dir3 , int pwmval3, int PWM3, int M3_ENCA, int M3_ENCB)
{
  analogWrite(PWM3, pwmval3);
  if (dir3 == 1) {
    digitalWrite(DIR_3, 1);
  }
  else if (dir3 == -1) { //LOW
    digitalWrite(DIR_3, 0);
  }
}

void setMotor4(int dir4 , int pwmval4, int PWM4, int M4_ENCA, int M4_ENCB)
{
  analogWrite(PWM4, pwmval4);
  if (dir4 == 1) {
    digitalWrite(DIR_4, 1);
  }
  else if (dir4 == -1) { //LOW
    digitalWrite(DIR_4, 0);
  }
}

void setMotor5(int dir5 , int pwmval5, int PWM5, int M5_ENCB)
{
  analogWrite(PWM5, pwmval5);
  if (dir5 == 1) {
    digitalWrite(DIR_5F, 1);
    digitalWrite(DIR_5B, 0);
  }
  else if (dir5 == -1) { //LOW
    digitalWrite(DIR_5F, 0);
    digitalWrite(DIR_5B, 1);
  }
}

/////////////////////////////////////////////////////////////////////////////encoder counts///////////////////////////////////////////////////////////////////////////////////////////////
ISR(PCINT0_vect) {
  if (PINB & B00000001) {
    int e = digitalRead(M5_ENCB);
    if (e > 0) {
      pos_5++;
    }
    else {
      pos_5--;
    }
  }
}

void M1update_encoder() {
  a = digitalRead(M1_ENCB);
  if (a > 0) {
    pos_1++;
  }
  else {
    pos_1--;
  }
}

void M2update_encoder() {
  b = digitalRead(M2_ENCB);
  if (b > 0) {
    pos_2++;
  }
  else {
    pos_2--;
  }
}

void M3update_encoder() {
  c = digitalRead(M3_ENCB);
  if (c > 0) {
    pos_3++;
  }
  else {
    pos_3--;
  }
}

void M4update_encoder() {
  d = digitalRead(M4_ENCB);
  if (d > 0) {
    pos_4++;
  }
  else {
    pos_4--;
  }
}
