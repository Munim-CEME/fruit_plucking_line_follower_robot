/************

* Last modified on 10 May 2023 by M. Munim SHafi
* Added back servo mechanism 
  #define BACK_SERVO_CLOSE backservo.write(4)
  #define BACK_SERVO_OPEN backservo.write(100)
* Added Pins for COLOR sensor
  #define S2_LEFT 38
  #define S3_LEFT 40
  #define OUT_LEFT 42
  #define OUT_RIGHT 44
  #define S3_RIGHT 46
  #define S2_RIGHT 50
* Added functions for processing green and red value
  int process_green_value(int, int, int);
  int process_red_value(int, int, int); 
* Added color sensing in all 3 stops
* only calibration required


************/

#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>
// #include <SharpIR.h>
// #include <MPU6050_light.h>
#include <Wire.h>

// definitions
// color define (Reverse for sensor array)
#define WHITE 1
#define BLACK 0

#define POT1 A13
#define POT2 A14
#define POT3 A15
// IR sensors front
// #define LEFTEST 23
// #define LEFTER 25
// #define MIDDLE 29
// #define RIGHTER 27
// #define RIGHTEST 31
#define LEFTEST 31//23
#define LEFTER 33//25
#define MIDDLE 27
#define RIGHTER 25//29      
#define RIGHTEST 23//31
// IR sensors Reverse
#define B_LEFTEST A1
#define B_LEFTER A0
#define B_MIDDLE A2
#define B_RIGHTER A3
#define B_RIGHTEST A4
// side IR Sensor
#define Side_sense 37
////// operating switches
#define LIMIT_SWITCH 39
#define BACK_LIMIT_SWITCH A6
#define Side_switch A5 //oringinal 34 - (A9 - 35 are external button options) 
#define onSwitch 32
#define Debug_Button 35
//#define RESET_BULB 33
// Motor direction and PWM definition
#define RIGHT_DIR 9
#define RIGHT_PWM 8
#define LEFT_DIR 4
#define LEFT_PWM 5
#define brake_R 52
#define brake_L 48

//Start button
#define START A7



//SERVO POSITIONS
#define LEFT_SERVO_OUT leftservo.write(180)
#define LEFT_SERVO_IN leftservo.write(30)
#define LEFT_SERVO_MID leftservo.write(88)
#define LEFT_SERVO_OFF leftservo.write(0)
#define RIGHT_SERVO_OUT rightservo.write(0)
#define RIGHT_SERVO_IN rightservo.write(180)
#define RIGHT_SERVO_MID rightservo.write(72)
#define RIGHT_SERVO_X rightservo.write(85)
#define BACK_SERVO_CLOSE backservo.write(4)
#define BACK_SERVO_OPEN backservo.write(100)

// INITIAL TURNS and SPEED delays for SENSOR based TURNS
#define NINTY_R 400
#define NINTY_L 400
#define ONE_EIGHTY_R 1200
#define ONE_EIGHTY_L 1200
#define INIT_SPEED 70
#define SENSE_SPEED_R 70
#define SENSE_SPEED_L 70

//TREE LINE FOLLOW PARAMETERS
#define TREE_LEFTER 55//65//45
#define TREE_LEFETEST 65//65//50
#define TREE_RIGHTER 55 //65//45
#define TREE_RIGHTEST 65//65 //50
#define TREE_BASE 65//40

//Constant for color sensor
#define PROCESS_TIME 10//15

//COLOR SENSOR PINS
#define S2_LEFT 38
#define S3_LEFT 40
#define OUT_LEFT 42
#define OUT_RIGHT 44
#define S3_RIGHT 46
#define S2_RIGHT 50

#define NEW_TURN_SPEED 100



//Signal Board inputs
int potpin = A13;
int potpin1 = A14;

//Old class modified into functions for time delays
// class RexDelay{
unsigned long long curr = 0;
unsigned long long prev = 0;
// public:
////Start timer
void startTimer() {
  prev = millis();
  // curr = millis();
}
// public:
////returns 1 when timer = stop time
bool istimesup(int stoptime) {
  curr = millis();
  if ((curr - prev) > stoptime) {
    // prev = curr;
    return 1;
  }
  return 0;
}

// };

// Servo Objects -> create servo object to control a servo
Servo rightservo;  //left servo
Servo leftservo;   //right servo
Servo backservo;  //back servo

//FUNCTIONS
void forward(int);
void forward(int,int);
void backwards(int);
void right_turn(int, int);
void left_turn(int, int);
void pause();
void pwm_zero();
void moveRight(int);
void moveLeft(int);
void print_sensor();
void moveRighter(int);
void moveLefter(int);
void line_follow();
void state_check();
void backline();
void E_brake_light();
void sense();
void E_brake_R();
void E_brake_L();
void E_brake();
void Speed_Cal_R();
void Speed_Cal_L();
void forward2(int, int);
void end();
void Ignore_stop();
void sensestop();
void statecheckStop();
void mechtest();
void debug_button();
// void Left_turn_sensor(int, int, int);
// void Right_turn_sensor(int, int, int);
int process_green_value(int, int, int);
int process_red_value(int, int, int);
void Left_turn_sensor_side(int angle, int speed);

//Modes
bool mod1;
bool mod2;
bool mod3;
bool mod4;
bool mod5;

// Speed defintion
float BASE = 0;
float BASE2 = 0;
float balancer = 0;
float TURN = 0;
float EDGE_TURN = 0;
float STOP_TURN = 5;
float DIFFERNTIAL = 40;
float OFFSET = 40;

// Delay time definition
#define turn_delay 450
#define stop_delay 300  //delay added for E-brake
#define stop_delay_light 180  //delay added for E-brake
#define short_turn_delay 60
#define back_delay 1500
#define for_delay 900

int counter = 0;

// variables for POTs
int POT_1 = 0;
int POT_2 = 0;
int POT_3 = 0;

//BOOL for turn
bool turn_flag = 0;

/// BOOL for sensers//
bool Leftest = 0;  // fornt IR sensors
bool Left = 0;
bool Center = 0;
bool Right = 0;
bool Rightest = 0;

bool side_sen = 0;

bool bLeftest = 0;  // Rear IR sensors
bool bLeft = 0;
bool bCenter = 0;
bool bRight = 0;
bool bRightest = 0;

// bool for state
bool S_forward = 0;
bool S_right = 0;
bool S_righter = 0;
bool S_left = 0;
bool S_lefter = 0;
bool S_counter = 0;

bool State = 0;
//Variable for limit switch
int switchVal = 0;
//Variable for testing servo values from potentiometer
int val;  // variable to read the value from the analog pin
int val1;


/// BOOL for side
bool side = 0;  //Toggle switch to decide arena

LiquidCrystal_I2C blue(0x27, 16, 2);
LiquidCrystal_I2C green(0x26, 16, 2);
//BOOL MUNIM
int endcounter = 0;
int startflag = 0;
int clawdelay = 700;
int start = 0;

bool forwardflag = 0;
bool turnflag = 0;
bool turnflag2 = 0;
int test;  //input variable for reading debug_button value
bool tree = 0;
bool back_line_follow = 0;
bool back_limit = -1;
bool clawflag;
bool ramp_flag = 1;

//Variables for Color Sensor
int red_left = 0;
int red_right = 0;
int green_left = 0;
int green_right = 0;
int threshold_green_left = 118;
int threshold_green_right = 84;
int threshold_red_left = 0;
int threshold_red_right = 0;
int combined_threshold_left = 25;//30;
int combined_threshold_right = 27;//17 ;


void setup() {
  Serial.begin(9600);
  if (side == 1) {
  }
  pinMode(RIGHT_DIR, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(LEFT_DIR, OUTPUT);
  pinMode(LEFT_PWM, OUTPUT);
  pinMode(brake_L, OUTPUT);
  pinMode(brake_R, OUTPUT);

  pinMode(LEFTEST, INPUT);
  pinMode(LEFTER, INPUT);
  pinMode(MIDDLE, INPUT);
  pinMode(RIGHTER, INPUT);
  pinMode(RIGHTEST, INPUT);

  pinMode(B_LEFTEST, INPUT);
  pinMode(B_LEFTER, INPUT);
  pinMode(B_MIDDLE, INPUT);
  pinMode(B_RIGHTER, INPUT);
  pinMode(B_RIGHTEST, INPUT);

  pinMode(S2_LEFT, OUTPUT);
  pinMode(S3_LEFT, OUTPUT);
  pinMode(S2_RIGHT, OUTPUT);
  pinMode(S3_RIGHT, OUTPUT);

  pinMode(OUT_LEFT, INPUT);
  pinMode(OUT_RIGHT, INPUT);

  digitalWrite(RIGHT_DIR, HIGH);
  digitalWrite(LEFT_DIR, HIGH);

  pinMode(Side_switch, INPUT_PULLUP);
  side = digitalRead(Side_switch);
  pinMode(LIMIT_SWITCH, INPUT_PULLUP);
  pinMode(BACK_LIMIT_SWITCH, INPUT_PULLUP);
  switchVal = digitalRead(LIMIT_SWITCH);
  back_limit = digitalRead(BACK_LIMIT_SWITCH);
  delay(500);
    pinMode(START, INPUT_PULLUP);
  start = digitalRead(START);
  while(digitalRead(START)==start){}
  backservo.attach(12);
  BACK_SERVO_CLOSE;
  // if(side==1){
    LEFT_SERVO_MID;
  // } else{
  //   LEFT_SERVO_OUT;
  // }
  RIGHT_SERVO_MID;
  
  leftservo.attach(11);
  delay(400);
  rightservo.attach(10);
  delay(100);  // attaches the servo on pin 9 to the servo object

  Serial.begin(9600);
  pinMode(Debug_Button, INPUT_PULLUP);
  
  forward(100);
  delay(300);
   side = digitalRead(Side_switch);
  // if(side == 1){
  //    LEFT_SERVO_OFF;
  //  }
  //  else {
   
  // //NOTHING TO DO
  // //  delay(100);
  // //  RIGHT_SERVO_MID;

  //   }



  // rightservo.write(180);
  // delay(100);
  // Serial.println(digitalRead(Side_switch));
}

// bool reset_flag = 1;
void loop(){
  //Reset bulb turns OFF after given time
//   if(millis()>1000 && reset_flag==1)
//   {
    
//     reset_flag = 0;
//   }

  sense();
  line_follow();

  switchVal = digitalRead(LIMIT_SWITCH);
//  if (S_counter == 1 && counter == -1) {
//         // First stop
//         Ignore_stop();
//     }

if (S_counter == 1 && counter == 0) {
        // First stop
        First_stop();
    }


if (S_counter == 1 && counter == 1) {

        // Second stop
        Second_stop();
    }

    if (S_counter == 1 && counter == 2) {
        Third_stop();

    }

    if(endcounter == 2){
      end();
    }

// forward(100);
}

// void loop() {
  
//   turn_flag = 1;
//   do {
//     if (turn_flag == 1) {
//       right_turn(70, 500);
//       turn_flag = 0;
//     } else {
//       digitalWrite(RIGHT_DIR, LOW);
//       digitalWrite(LEFT_DIR, HIGH);
//       digitalWrite(brake_L, HIGH);
//       digitalWrite(brake_R, LOW);
//       analogWrite(RIGHT_PWM, 50);
//       analogWrite(LEFT_PWM, 50);
      
//     }
//     Center = digitalRead(MIDDLE);
    
    
//     if(Center == 0) break;
//   } while (1);
//   // left_turn(50, 340);
//   E_brake();
//   // digitalWrite(RIGHT_PWM, 0);
//   // digitalWrite(LEFT_PWM, 0);
//   delay(1000);
// }

void Ignore_stop() {
}

void First_stop() {
  counter++;
  endcounter++;
  // blue.clear();
  // blue.print("First Stop");
  //simpleBrakeF();
  //delay(275);
  E_brake();

  //Move till left sensor detects black line
  while (digitalRead(Side_sense) == BLACK) {
    // sensestop();
    // line_follow();
    forward(40);
  }
  E_brake();
  //simpleBrakeF();
  // backwards(30);
  // delay(275);
  //delay(1000);

  if (side == 1) {
    // left_turn(STOP_TURN, (turn_delay / 2) + 15);
    // Left_turn_sensor(NINTY_L, INIT_SPEED, SENSE_SPEED_L);
        Left_turn_sensor(90,NEW_TURN_SPEED);

  } else {
    // right_turn(STOP_TURN, turn_delay / 1.75);
    // Right_turn_sensor(NINTY_R, INIT_SPEED, SENSE_SPEED_R);
        Right_turn_sensor(90,NEW_TURN_SPEED);

  }
 
  // if(side==1){
     LEFT_SERVO_OUT;
   RIGHT_SERVO_OUT;
  // }else{}
    E_brake();
    delay(50);
  // if (side == 0) {
  //   backwards(40);
  //   delay(250);
  //   LEFT_SERVO_OUT;
  //   RIGHT_SERVO_OUT;
  //    delay(250);
  // }
  // else{}
   
  //delay(250);
  // backwards(30);
  // delay(500);
  // E_brake();
  // LEFT_SERVO_OUT;
  // delay(200);
  // RIGHT_SERVO_OUT;
  // //  rightservo.write(0);
  // delay(100);
  // E_brake();
  //delay(1000);
  // debug_button();
  tree = 1;
  forward(40);
  
  while (switchVal != 0) {
    switchVal = digitalRead(LIMIT_SWITCH);
    sense();
    line_follow();
  }
  tree = 0;
  // E_brake();
  pwm_zero();
  green_left = process_green_value( S2_LEFT, S3_LEFT, OUT_LEFT);
  red_left = process_red_value(S2_LEFT, S3_LEFT, OUT_LEFT);
  green_right = process_green_value( S2_RIGHT, S3_RIGHT, OUT_RIGHT); 
  red_right = process_red_value(S2_RIGHT, S3_RIGHT, OUT_RIGHT);
  if(green_left - red_left < -1*combined_threshold_left || green_left - red_left > combined_threshold_left){
     LEFT_SERVO_MID;
     clawflag=1;
     
    //delay(500);
  }
  else{}

  if(green_right - red_right < -1*combined_threshold_right || green_right - red_right > combined_threshold_right){
    RIGHT_SERVO_MID;
    clawflag = 1;
    
   // delay(500);
  }
  else{}
  if(clawflag == 1){
     delay( clawdelay);
     clawflag = 0;
  }
  else{}
  
  //  rightservo.write(70);
  // delay(5000);
 // E_brake();
  //delay(1000);
  // debug_button();

  //delay(1000);
  //delay(1000);
  delay(30);
  while (digitalRead(Side_sense) == BLACK) {
    // sensestop();
    // line_follow();
    backwards(40);
  }
   pwm_zero();
  // backwards(60);
  // delay(600);
  // E_brake();
  //delay(1000);
  
  // LEFT_SERVO_OUT;
  // delay(200);
  RIGHT_SERVO_MID;
  delay(200);
  LEFT_SERVO_MID;

  backwards(50);
  delay(50);
 
  // rightservo.write(180);
  //delay(200);


  // if (endcounter == 1) {
  //   if (side == 1) {
  //     // right_turn(STOP_TURN,turn_delay);
  //     // left_turn(STOP_TURN, (turn_delay) + 10);
  //     Left_turn_sensor(ONE_EIGHTY_L, INIT_SPEED, SENSE_SPEED_L);
  //   } else {
  //     // left_turn(STOP_TURN, turn_delay);
  //     Right_turn_sensor(ONE_EIGHTY_R, INIT_SPEED, SENSE_SPEED_R);
  //   }


    //E_brake();
    back_line_follow = 1;
  
  //  forward(BASE);
  //   sense();
  //   delay(10000);
}

// void Second_stop(){
//   counter++;
//             blue.clear();
//             blue.print("Second Stop");
//             pause();
//             while (digitalRead(Side_sense) == BLACK) {
//                 forward(80);
//             }
//             pwm_zero();

//             if (side == 1) {
//                 right_turn(TURN,turn_delay);
//             } else {
//                 left_turn(TURN,turn_delay);
//             }

//             delay(turn_delay+80);
//             pwm_zero();
//             //E_brake(1,1);
//             delay(stop_delay);
//            // mechanism();
//             while (S_counter == 1) {

//                 forward(BASE);
//             }
//             delay(40);
//             pause();

//             pwm_zero();
//             delay(stop_delay);
//             if (side == 1) {
//                 left_turn(TURN,turn_delay);
//             } else {
//                 right_turn(TURN,turn_delay);
//             }
//             delay(turn_delay+160);
//             pwm_zero();
// //            delay(stop_delay);
//             blue.setCursor(0, 1);
//             blue.print("Forward");
//             blue.setCursor(0, 0);
//             forward(BASE);
//             sense();
// }
void Second_stop() {
  back_line_follow = 0;
  counter++;
  blue.clear();
  blue.print("First Stop");
  E_brake();
  //delay(1000);
  while (digitalRead(Side_sense) == BLACK) {
    // sensestop();
    // line_follow();
    backwards(40);
  }
  E_brake();
  //delay(1000);
  // backwards(30);
  // delay(200);
  // E_brake();
  //delay(1000);

  // if (side == 1) {
  //   // left_turn(STOP_TURN,turn_delay/1.5);
  //   left_turn(STOP_TURN, (turn_delay / 2) + 25);
  // } else {
  //   right_turn(STOP_TURN, turn_delay / 1.5);
  // }

  if (side == 1) {
    // left_turn(STOP_TURN, (turn_delay / 2) + 15);
    // Right_turn_sensor(NINTY_R, INIT_SPEED, SENSE_SPEED_R);
        Right_turn_sensor(90,NEW_TURN_SPEED);

  } else {
    // right_turn(STOP_TURN, turn_delay / 1.75);
    
    // Left_turn_sensor(NINTY_L, INIT_SPEED, SENSE_SPEED_L);
        Left_turn_sensor(90,NEW_TURN_SPEED);

  }
  // E_brake();
  // delay(turn_delay);
  // E_brake();
  // delay(stop_delay);

  // blue.setCursor(0, 1);
  // blue.print("Forward");
  // blue.setCursor(0, 0);
  // forward(BASE - 50);
  // delay(for_delay);
  LEFT_SERVO_OUT;
  RIGHT_SERVO_OUT;
  E_brake();
  delay(50);
  // backwards(30);
  // delay(300);
  // E_brake();
  //delay(500);

  //E_brake();
  // LEFT_SERVO_OUT;
  // RIGHT_SERVO_OUT;
  //  rightservo.write(0);
  // delay(500);

  //leftservo.write(180);
  //delay(50);
  // debug_button();
  //E_brake();
  //delay(1000);
  //E_brake();
  tree = 1;
  forward(40);
  while (switchVal != 0) {
    switchVal = digitalRead(LIMIT_SWITCH);
    sense();
    line_follow();
  }
  pwm_zero();
  tree = 0;
  //delay(300);

  // E_brake();
  // delay(1000);
  // debug_button();
green_left = process_green_value( S2_LEFT, S3_LEFT, OUT_LEFT);
  red_left = process_red_value(S2_LEFT, S3_LEFT, OUT_LEFT);
  green_right = process_green_value( S2_RIGHT, S3_RIGHT, OUT_RIGHT); 
  red_right = process_red_value(S2_RIGHT, S3_RIGHT, OUT_RIGHT);
  if(green_left - red_left < -1*combined_threshold_left || green_left - red_left > combined_threshold_left){
     LEFT_SERVO_MID;
     clawflag = 1;
    //delay(500);
  }
  else{}

  if(green_right - red_right < -1*combined_threshold_right || green_right - red_right > combined_threshold_right){
    RIGHT_SERVO_MID;
    clawflag = 1;
    //delay(500);
  }
  else{}
  // rightservo.write(70);
 if(clawflag == 1){
     delay(clawdelay);
     clawflag = 0;
  }
  else{}
  while (digitalRead(Side_sense) == BLACK) {
    // sensestop();
    // line_follow();
    backwards(40);
  }
  E_brake();
  //delay(1000);
  // RIGHT_SERVO_OUT;
  // delay(500);
  // LEFT_SERVO_OFF;
  // //  rightservo.write(180);
  // delay(1000);
  //  if (side == 1) {
  //  RIGHT_SERVO_OUT;
  // delay(300);
  // LEFT_SERVO_OFF;
  // //delay(500);
  // }
  // else {
  
  // LEFT_SERVO_OUT;
  // delay(300);
  // RIGHT_SERVO_IN;
  // //delay(500);
  // }
  LEFT_SERVO_MID;
  RIGHT_SERVO_MID;


  //right_turn(TURN,turn_delay/2);
  if (side == 1) {
    // right_turn(STOP_TURN, turn_delay / 2);
    // Right_turn_sensor(NINTY_R, INIT_SPEED, SENSE_SPEED_R);
        Right_turn_sensor(90,NEW_TURN_SPEED);

  } else {
    // left_turn(STOP_TURN, turn_delay / 1.5);
    //  Left_turn_sensor(NINTY_L, INIT_SPEED, SENSE_SPEED_L);
        Left_turn_sensor(90,NEW_TURN_SPEED);

  }
  E_brake();

  // RIGHT_SERVO_OUT;
  // delay(1000);
  forward(BASE);
  sense();
}
void Third_stop() {
  // Third stop
  counter++;
  endcounter++;
  blue.clear();
  blue.print("Third Stop");
  E_brake();
  //delay(1000);
  while (digitalRead(Side_sense) == BLACK) {
    // sensestop();
    // line_follow();
    // sense();
    // line_follow();
    forward(40);
  }
  E_brake();
  // backwards(30);
  // delay(200);
   //E_brake();
  //delay(1000);
  //delay(1000);
  // backwards(30);
  // delay(100);

  // if (side == 1) {
  //   left_turn(STOP_TURN, (turn_delay / 2) + 10);
  // } else {
  //   right_turn(STOP_TURN, turn_delay / 2.25);
  // }

  if (side == 1) {
    // left_turn(STOP_TURN, (turn_delay / 2) + 15);
    //  Left_turn_sensor(NINTY_L, INIT_SPEED, SENSE_SPEED_L);
       Left_turn_sensor(90,NEW_TURN_SPEED);

  } else {
    // right_turn(STOP_TURN, turn_delay / 1.75);
    // Right_turn_sensor(NINTY_R, INIT_SPEED, SENSE_SPEED_R);
        Right_turn_sensor(90,NEW_TURN_SPEED);

  }
   LEFT_SERVO_OUT;
  RIGHT_SERVO_OUT;
  E_brake();
  delay(50);

  // backwards(30);
  // delay(300);
  //E_brake();
  // E_brake();
  //delay(1000);
  // debug_button();
 
  // //  rightservo.write(0);
  delay(300); // to delete
  tree = 1;
  forward(40);
  while (switchVal != 0) {
    switchVal = digitalRead(LIMIT_SWITCH);
    sense();
    line_follow();
  }
  pwm_zero();
  tree = 0;
  //E_brake();
  //delay(500);
  // debug_button();
  green_left = process_green_value( S2_LEFT, S3_LEFT, OUT_LEFT);
  red_left = process_red_value(S2_LEFT, S3_LEFT, OUT_LEFT);
  green_right = process_green_value( S2_RIGHT, S3_RIGHT, OUT_RIGHT); 
  red_right = process_red_value(S2_RIGHT, S3_RIGHT, OUT_RIGHT);
  if(green_left - red_left < -1*combined_threshold_left || green_left - red_left > combined_threshold_left){
     LEFT_SERVO_MID;
      clawflag = 1;
    //delay(500);
  }
  else{}

  if(green_right - red_right < -1*combined_threshold_right || green_right - red_right > combined_threshold_right){
    RIGHT_SERVO_MID;
    clawflag = 1;
    //delay(500);
  }
  else{}

  if(clawflag == 1){
     delay(clawdelay);
     clawflag = 0;
  }
  else{}
 
  //debug_button();
  while (digitalRead(Side_sense) == BLACK) {
    // sensestop();
    // line_follow();
    backwards(40);
  }
  delay(100);
  E_brake();
  //delay(500);
  //  if(endcounter == 1){
  //   if(side == 1){
  //     right_turn(STOP_TURN,turn_delay);
  //   }
  //   else {
  //   left_turn(STOP_TURN,turn_delay);
  //   }
  //  }
}

void calibrate() {
  POT_1 = (float)analogRead(POT1) / 4;
  POT_2 = ((float)analogRead(POT2)) / 4;
  POT_3 = (float)analogRead(POT3) / 4;

  delay(6000);
}



void forward(int in_speed) {  // print what it is doing
  blue.setCursor(0, 1);
  blue.print("        ");
  blue.setCursor(0, 1);
  blue.print("FORWARD");
  // actual function
  //delay(30);
  digitalWrite(RIGHT_DIR, HIGH);
  digitalWrite(brake_R, HIGH);
  //delay(30);
  digitalWrite(LEFT_DIR, HIGH);
  digitalWrite(brake_L, HIGH);
  // to be optimized using signal board
  if(ramp_flag == 1){
    for(int i = 30; i<=in_speed; i=i+10){
      analogWrite(RIGHT_PWM, i);
      analogWrite(LEFT_PWM, i);
    }
    ramp_flag = 0;
  }else{
  analogWrite(RIGHT_PWM, in_speed);
  analogWrite(LEFT_PWM, in_speed);
  }
}

void forward(int left, int right) {  // print what it is doing
  blue.setCursor(0, 1);
  blue.print("        ");
  blue.setCursor(0, 1);
  blue.print("FORWARD");
  // actual function
  //delay(30);
  digitalWrite(RIGHT_DIR, HIGH);
  digitalWrite(brake_R, HIGH);
  //delay(30);
  digitalWrite(LEFT_DIR, HIGH);
  digitalWrite(brake_L, HIGH);
  // to be optimized using signal board
  analogWrite(RIGHT_PWM, right);
  analogWrite(LEFT_PWM, left);
}


void forward2(int in_speed, int s2) {  // print what it is doing
  blue.setCursor(0, 1);
  blue.print("        ");
  blue.setCursor(0, 1);
  blue.print("FORWARD");
  // actual function
  E_brake();
  digitalWrite(RIGHT_DIR, HIGH);
  digitalWrite(brake_R, HIGH);
  digitalWrite(LEFT_DIR, HIGH);
  digitalWrite(brake_L, HIGH);
  // to be optimized using signal board
  analogWrite(RIGHT_PWM, in_speed);
  analogWrite(LEFT_PWM, s2);
}

void backwards(int in_speed) {  // print what it is doing
  blue.setCursor(0, 1);
  blue.print("        ");
  blue.setCursor(0, 1);
  blue.print("BACKWARD");
  ///
  // E_brake();
  //delay(30);
  digitalWrite(RIGHT_DIR, LOW);
  digitalWrite(brake_R, LOW);
  //delay(30);
  digitalWrite(LEFT_DIR, LOW);
  digitalWrite(brake_L, LOW);
  
  // als to be optimized using signal board

    if(ramp_flag == 1){
    for(int i = 30; i<=in_speed; i=i+10){
      analogWrite(RIGHT_PWM, i);
      analogWrite(LEFT_PWM, i);
    }
    ramp_flag = 0;
  }else{
  analogWrite(RIGHT_PWM, in_speed);
  analogWrite(LEFT_PWM, in_speed);
  }
}

void right_turn(int turn_speed, int degree) {  // Printing that it is turning right

  blue.setCursor(0, 1);
  blue.print("        ");
  blue.setCursor(0, 1);
  blue.print("R---Turn");
  digitalWrite(RIGHT_DIR, LOW);
  digitalWrite(LEFT_DIR, HIGH);
  digitalWrite(brake_L, HIGH);
  digitalWrite(brake_R, LOW);
  analogWrite(RIGHT_PWM, turn_speed);
  analogWrite(LEFT_PWM, turn_speed);
  delay(degree);
  //pwm_zero();
}

void left_turn(int turn_speed, int degree) {  // Printing that it is turning Left
  blue.setCursor(0, 1);
  blue.print("        ");
  blue.setCursor(0, 1);
  blue.print("L---Turn");
  /// actually turning Left
  digitalWrite(RIGHT_DIR, HIGH);
  digitalWrite(LEFT_DIR, LOW);
  digitalWrite(brake_L, LOW);
  digitalWrite(brake_R, HIGH);
  analogWrite(RIGHT_PWM, turn_speed);
  analogWrite(LEFT_PWM, turn_speed);
  delay(degree);
  // pwm_zero();
}

void moveRight(int speed) {
  // try turning of the motor instead of reversing it
  blue.setCursor(0, 1);
  blue.print("        ");
  blue.setCursor(0, 1);
  blue.print("Right");
  digitalWrite(RIGHT_DIR, HIGH);  //this
  digitalWrite(LEFT_DIR, HIGH);
  digitalWrite(brake_L, HIGH);
  digitalWrite(brake_R, HIGH);
  analogWrite(RIGHT_PWM, DIFFERNTIAL);  ///and this
  analogWrite(LEFT_PWM, speed);
}

void moveRight_B(int speed) {
  // try turning of the motor instead of reversing it
  blue.setCursor(0, 1);
  blue.print("        ");
  blue.setCursor(0, 1);
  blue.print("Right");
  digitalWrite(RIGHT_DIR, LOW);  //this
  digitalWrite(LEFT_DIR, LOW);
  digitalWrite(brake_L, LOW);
  digitalWrite(brake_R, LOW);
  analogWrite(RIGHT_PWM, DIFFERNTIAL);  ///and this
  analogWrite(LEFT_PWM, speed);
}

void moveLeft(int speed) {
  // try turning of the motor instead of reversing it
  blue.setCursor(0, 1);
  blue.print("        ");
  blue.setCursor(0, 1);
  blue.print("Left");
  digitalWrite(RIGHT_DIR, HIGH);
  digitalWrite(LEFT_DIR, HIGH);
  digitalWrite(brake_R, HIGH);
  digitalWrite(brake_L, HIGH);
  analogWrite(RIGHT_PWM, speed);
  analogWrite(LEFT_PWM, DIFFERNTIAL);
}

void moveLeft_B(int speed) {
  // try turning of the motor instead of reversing it
  blue.setCursor(0, 1);
  blue.print("        ");
  blue.setCursor(0, 1);
  blue.print("Left");
  digitalWrite(RIGHT_DIR, LOW);
  digitalWrite(LEFT_DIR, LOW);
  digitalWrite(brake_R, LOW);
  digitalWrite(brake_L, LOW);
  analogWrite(RIGHT_PWM, speed);
  analogWrite(LEFT_PWM, DIFFERNTIAL);
}

void moveRighter(int speed) {
  blue.setCursor(0, 1);
  blue.print("        ");
  blue.setCursor(0, 1);
  blue.print("Righter");
  digitalWrite(RIGHT_DIR, LOW);  //Reverse
  digitalWrite(LEFT_DIR, HIGH);  //Forward
  digitalWrite(brake_R, LOW);
  digitalWrite(brake_L, HIGH);
  analogWrite(RIGHT_PWM, speed - OFFSET);
  analogWrite(LEFT_PWM, speed + OFFSET);
}

void moveRighter_B(int speed) {
  // blue.setCursor(0, 1);
  // blue.print("        ");
  // blue.setCursor(0, 1);
  // blue.print("Righter");
  digitalWrite(RIGHT_DIR, LOW);  //Reverse
  digitalWrite(LEFT_DIR, HIGH);  //Forward
  digitalWrite(brake_R, LOW);
  digitalWrite(brake_L, HIGH);
  analogWrite(RIGHT_PWM, speed + OFFSET);
  analogWrite(LEFT_PWM, speed - OFFSET);
}

void moveLefter(int speed) {
  blue.setCursor(0, 1);
  blue.print("        ");
  blue.setCursor(0, 1);
  blue.print("Lefter");
  digitalWrite(RIGHT_DIR, HIGH);
  digitalWrite(LEFT_DIR, LOW);  //Reverse
  digitalWrite(brake_R, HIGH);
  digitalWrite(brake_L, LOW);
  analogWrite(RIGHT_PWM, speed + OFFSET);
  analogWrite(LEFT_PWM, speed - OFFSET);
}

void moveLefter_B(int speed) {
  blue.setCursor(0, 1);
  blue.print("        ");
  blue.setCursor(0, 1);
  blue.print("Lefter");
  digitalWrite(RIGHT_DIR, HIGH);
  digitalWrite(LEFT_DIR, LOW);  //Reverse
  digitalWrite(brake_R, HIGH);
  digitalWrite(brake_L, LOW);
  analogWrite(RIGHT_PWM, speed - OFFSET);
  analogWrite(LEFT_PWM, speed + OFFSET);
}


void moveRighter1(int speed) {
  blue.setCursor(0, 1);
  blue.print("        ");
  blue.setCursor(0, 1);
  blue.print("Righter");
  digitalWrite(RIGHT_DIR, LOW);  //Reverse
  digitalWrite(LEFT_DIR, HIGH);  //Forward
  digitalWrite(brake_R, LOW);
  digitalWrite(brake_L, HIGH);
  analogWrite(RIGHT_PWM, 30);//0
  analogWrite(LEFT_PWM, speed);
}

void moveLefter1(int speed) {
  blue.setCursor(0, 1);
  blue.print("        ");
  blue.setCursor(0, 1);
  blue.print("Lefter");
  digitalWrite(RIGHT_DIR, HIGH);
  digitalWrite(LEFT_DIR, LOW);  //Reverse
  digitalWrite(brake_R, HIGH);
  digitalWrite(brake_L, LOW);
  analogWrite(RIGHT_PWM, speed);
  analogWrite(LEFT_PWM, 30);//0
}
void Left_turn_sensor(int angle, int speed) {
  int last = 0;
  int double_skip = 0;
  // turn_flag = 1;
  int current = -1;
      digitalWrite(RIGHT_DIR, HIGH);
      digitalWrite(LEFT_DIR, LOW);
      digitalWrite(brake_L, LOW);
      digitalWrite(brake_R, HIGH);
      analogWrite(RIGHT_PWM, speed);
      analogWrite(LEFT_PWM, speed);
      // delay(100);
  while(1){
    Rightest = digitalRead(RIGHTEST);
    if (Rightest == 0) break;
  }
  while(1) {
  //   if (turn_flag == 1) {
  //     left_turn(init_speed, angle);
  //     turn_flag = 0;
  //   } else {
      // digitalWrite(RIGHT_DIR, HIGH);
      // digitalWrite(LEFT_DIR, LOW);
      // digitalWrite(brake_L, LOW);
      // digitalWrite(brake_R, HIGH);
      // analogWrite(RIGHT_PWM, speed);
      // analogWrite(LEFT_PWM, speed);
      
  //   }
    //sense();
    Left = digitalRead(LEFTER);
    Leftest = digitalRead(LEFTEST);
    Center = digitalRead(MIDDLE);
    Right = digitalRead( RIGHTER);
    Rightest = digitalRead( RIGHTEST);

    // if(Leftest == 0){
    //   double_skip = 1;
    //   break;
    // } else if(Left == 0){
    //   double_skip = 1;
    //   break;
    // } else if(Center == 0){
    //   double_skip = 0;
    //   break;
    // } else if(Right == 0){
    //   double_skip = 0;
    //   break;
    // } else if(Rightest == 0){
    //   double_skip = 0;
    //   break;
    // } else{
    //   continue;
    // }   
    if(Left== HIGH && Leftest==HIGH && Center== HIGH && Rightest ==HIGH && Right== HIGH && current == -1){ current = digitalRead(MIDDLE);} 
    if(current!=-1) current = digitalRead(MIDDLE);
    if(current == 0 && last == 1) break;
    last = current;
  }
  last = 0;
  current=0;
  if (angle == 180){
    while(1) {
    if(Left== HIGH && Leftest==HIGH && Center== HIGH && Rightest ==HIGH && Right== HIGH && current == -1) current = digitalRead(MIDDLE);
    if(current!=-1) current = digitalRead(MIDDLE);
    if(current == 0 && last == 1) break;
    last = current;
    }
  }
}

void Right_turn_sensor(int angle, int speed) {
  int last = 0;
  int double_skip = 0;
  // turn_flag = 1;
  int current = -1;
       digitalWrite(RIGHT_DIR, LOW);
      digitalWrite(LEFT_DIR, HIGH);
      digitalWrite(brake_L, HIGH);
      digitalWrite(brake_R, LOW);
      analogWrite(RIGHT_PWM, speed+10);
      analogWrite(LEFT_PWM, speed);
      // delay(100);
  while(1){
    Leftest = digitalRead(LEFTEST);
    if (Leftest == 0) break;
  }
  while(1) {
  //   if (turn_flag == 1) {
  //     left_turn(init_speed, angle);
  //     turn_flag = 0;
  //   } else {
      // digitalWrite(RIGHT_DIR, HIGH);
      // digitalWrite(LEFT_DIR, LOW);
      // digitalWrite(brake_L, LOW);
      // digitalWrite(brake_R, HIGH);
      // analogWrite(RIGHT_PWM, speed);
      // analogWrite(LEFT_PWM, speed);
      
  //   }
    //sense();
    Left = digitalRead(LEFTER);
    Leftest = digitalRead(LEFTEST);
    Center = digitalRead(MIDDLE);
    Right = digitalRead( RIGHTER);
    Rightest = digitalRead( RIGHTEST);
    
    // if(Leftest == HIGH){
    //   double_skip = 0;
    //   break;
    // } else if(Left == HIGH){
    //   double_skip = 0;
    //   break;
    // } else if(Center == HIGH){
    //   double_skip = 0;
    //   break;
    // } else if(Right == HIGH){
    //   double_skip = 1;
    //   break;
    // } else if(Rightest == HIGH){
    //   double_skip = 1;
    //   break;
    // } else{
    //   continue;
    // }
   

    if(Left== HIGH && Leftest==HIGH && Center== HIGH && Rightest ==HIGH && Right== HIGH && current == -1){ current = digitalRead(MIDDLE);} 
    if(current!=-1) current = digitalRead(MIDDLE);
    if(current == 0 && last == 1) {break;}
    last = current;
  }
  last = 0;
  current=0;
  if (angle == 180){
    while(1) {
    if(Left== HIGH && Leftest==HIGH && Center== HIGH && Rightest ==HIGH && Right== HIGH && current == -1) current = digitalRead(MIDDLE);
    if(current!=-1) current = digitalRead(MIDDLE);
    if(current == 0 && last == 1) {break;}
    last = current;
    }
  }
  // left_turn(50, 340);
  // E_brake();
  // digitalWrite(RIGHT_PWM, 0);
  // digitalWrite(LEFT_PWM, 0);
  // delay(1000);
}


void Left_turn_sensor_j2(int turn_time, int init_speed, int sense_speed) {
  
  turn_flag = 1;
  int right = -1;
  do {
    if (turn_flag == 1) {
      left_turn(init_speed, turn_time);
      turn_flag = 0;
    } else {
      digitalWrite(RIGHT_DIR, HIGH);
      digitalWrite(LEFT_DIR, LOW);
      digitalWrite(brake_L, LOW);
      digitalWrite(brake_R, HIGH);
      analogWrite(RIGHT_PWM, sense_speed);
      analogWrite(LEFT_PWM, sense_speed);
      
    }
    right = digitalRead(MIDDLE);
    
    
    if(right == 0) break;
  } while (1);
  // left_turn(50, 340);
  // E_brake();
  // digitalWrite(RIGHT_PWM, 0);
  // digitalWrite(LEFT_PWM, 0);
  // delay(1000);
}

// void line_follow() {
//   if (S_lefter) {
//     //   if(forwardflag == 1){
//     //   E_brake_Turn();
//     //   forwardflag = 0;
//     // }

//     //  turnflag = 1;
//     if (tree == 1) {
//       moveLefter1(TREE_LEFETEST);
//     } else {
//       moveLefter(EDGE_TURN);
//     }




//   } else if (S_left) {
//     //  turnflag = 1;
//     // if(forwardflag == 1){
//     //   E_brake_Turn();
//     //   forwardflag = 0;
//     // }
//     // if(turnflag == 1){
//     //   E_brake_Turn();
//     //   turnflag = 0;
//     // }
//     if (tree == 1) {
//       moveLefter1(TREE_LEFTER);
//     } else {
//       moveLeft(TURN);
//     }



//   } else if (S_forward) {
//     // if(turnflag == 1){
//     //   E_brake_Turn();
//     //   turnflag = 0;
//     // }

//     if(tree == 1){
//       forward(TREE_BASE);
//     }
//     else{
//         forwardflag = 1;
//         forward(BASE);
//     }
    


//   } else if (S_right) {
//     // turnflag = 1;
//     //   if(forwardflag == 1){
//     //   E_brake_Turn();
//     //   forwardflag = 0;
//     // }

//     // if(turnflag == 1){
//     //   E_brake_Turn();
//     //   turnflag = 0;
//     // }
//     if (tree == 1) {
//       moveRighter1(50);
//     } else {
//       moveRight(TREE_RIGHTER);
//     }


//   } else if (S_righter) {
//     //   if(forwardflag == 1){
//     //   E_brake_Turn();
//     //   forwardflag = 0;
//     // }
//     // turnflag = 1;
//     if (tree == 1) {
//       moveRighter1(55);
//     } else {
//       moveRighter(TREE_RIGHTEST);
//     }
//   }
// }
void line_follow() {
  if (S_lefter) {
    //   if(forwardflag == 1){
    //   E_brake_Turn();
    //   forwardflag = 0;
    // }

    //  turnflag = 1;
    if (back_line_follow == 1) {
       moveLefter_B(EDGE_TURN);
    
    }
    else {
    
    if (tree == 1) {
        moveLefter1(TREE_LEFETEST);
      } else {
        moveLefter(EDGE_TURN);
      }
    }
    

  } else if (S_left) {
    //  turnflag = 1;
    // if(forwardflag == 1){
    //   E_brake_Turn();
    //   forwardflag = 0;
    // }
    // if(turnflag == 1){
    //   E_brake_Turn();
    //   turnflag = 0;
    // }
    if (back_line_follow == 1) {
       moveRight_B(TURN);
    
    }
   else {
    if (tree == 1) {
      moveLefter1(TREE_LEFTER);
    } else {
      moveLeft(TURN);
    }
   }



  } else if (S_forward) {
    // if(turnflag == 1){
    //   E_brake_Turn();
    //   turnflag = 0;
    // }
    if (back_line_follow == 1) {
       backwards(BASE);
       forwardflag = 1;
    
    }
   else {

    if(tree == 1){
      forward(TREE_BASE);//original 40
    }
    else{
        forwardflag = 1;
        forward(BASE);
    }
   }
    


  } else if (S_right) {
    // turnflag = 1;
    //   if(forwardflag == 1){
    //   E_brake_Turn();
    //   forwardflag = 0;
    // }

    // if(turnflag == 1){
    //   E_brake_Turn();
    //   turnflag = 0;
    // }
    if (back_line_follow == 1) {
       moveLeft_B(TURN);
    
    }
    else{
    if (tree == 1) {
      moveRighter1(TREE_RIGHTER);
    } else {
      moveRight(TURN);
    }
    }

  } else if (S_righter) {
    //   if(forwardflag == 1){
    //   E_brake_Turn();
    //   forwardflag = 0;
    // }
    // turnflag = 1;
     if (back_line_follow == 1) {
       moveRighter_B(EDGE_TURN);
    
    }
    else{
    if (tree == 1) {
      moveRighter1(TREE_RIGHTEST);
    } else {
      moveRighter(EDGE_TURN);
    }
    }
  }
}




void sense() {
  State = digitalRead(onSwitch);  //UNNECESSARY CODE !!!!

  if(back_line_follow == 1){
    Leftest = digitalRead(B_LEFTEST);
  Left = digitalRead(B_LEFTER);
  Center = digitalRead(B_MIDDLE);
  Right = digitalRead(B_RIGHTER);
  Rightest = digitalRead(B_RIGHTEST);
  }
  else{
     Leftest = digitalRead(LEFTEST);
  Left = digitalRead(LEFTER);
  Center = digitalRead(MIDDLE);
  Right = digitalRead(RIGHTER);
  Rightest = digitalRead(RIGHTEST);

  }

  side_sen = digitalRead(Side_sense);
  //  side = digitalRead(Side_switch);
   Serial.println(MIDDLE);
  

  // POT_1 = (float)analogRead(POT1) / 4;
  // POT_2 = ((float)analogRead(POT2)) / 4;
  // POT_3 = (float)analogRead(POT3) / 4;
  // POT_1 = 50;
  // POT_2 = 40;
  // POT_3 = 40;
  // POT_1 = (float)analogRead(POT1) / 4;
  // POT_2 = ((float)analogRead(POT2)) / 4;
  // POT_3 = (float)analogRead(POT3) / 4;


  // Serial.print("POT1:");
  // Serial.print(POT_1);
  // Serial.print(" ");
  // Serial.print("POT2:");
  // Serial.print(POT_2);
  // Serial.print(" ");
  // Serial.print("POT3:");
  // Serial.println(POT_3);

  //front_distance = Sharp_Front.getDistance();

  // gyrosense();
  //BASE = POT_1;

  // BASE = POT_1;
  // //balancer = POT_2/(float)(255);
  // TURN = 255*(POT_2/(float)(255));
  // EDGE_TURN = 255*(POT_3/(float)(255));

  if (side == 0) {

    if (counter == 1) {
      // BASE = 60;
      // TURN = 150;
      // EDGE_TURN = 65;
      // DIFFERNTIAL = 30;
      // OFFSET = 30;
      BASE = 80;
      TURN = 90;
      EDGE_TURN = 65;
      DIFFERNTIAL = 40;
      OFFSET = 20;
    }

    else {
      // BASE = 70;
      // TURN = 130;
      // EDGE_TURN = 55;
      // DIFFERNTIAL = 30;
      // OFFSET = 30;
       BASE = 80;
      TURN = 110;
      EDGE_TURN = 75;
      DIFFERNTIAL = 40;
      OFFSET = 20;
    }
  }

  else {
    if (counter == 1) {
      BASE = 90; //90
      TURN = 90; //100
      EDGE_TURN = 60;
      DIFFERNTIAL = 50;
      OFFSET = 20;//30;
    }

    else {
      BASE = 90; //90
      TURN = 100; //100
      EDGE_TURN = 65;
      DIFFERNTIAL = 50;
      OFFSET = 20;//30;
    }
  }


  state_check();
}

void state_check() {
  S_counter = 0;
  S_forward = 0;
  S_left = 0;
  S_lefter = 0;
  S_right = 0;
  S_righter = 0;

  if (  // check for forward ref my notes.
    (
      // (Leftest == WHITE && Left == WHITE && Center == WHITE && Right == WHITE && Rightest == WHITE)    // 0 0 0 0 0    ///////////WHY???????????
      //   ||
      (Leftest == WHITE && Left == WHITE && Center == BLACK && Right == WHITE && Rightest == WHITE)     // 0 0 1 0 0
      || (Leftest == WHITE && Left == WHITE && Center == BLACK && Right == WHITE && Rightest == BLACK)  // 0 0 1 0 1
      || (Leftest == WHITE && Left == BLACK && Center == WHITE && Right == WHITE && Rightest == BLACK)  // 0 1 0 0 1
      || (Leftest == WHITE && Left == BLACK && Center == WHITE && Right == BLACK && Rightest == WHITE)  // 0 1 0 1 0
      || (Leftest == WHITE && Left == BLACK && Center == WHITE && Right == BLACK && Rightest == BLACK)  // 0 1 0 1 1
      || (Leftest == BLACK && Left == WHITE && Center == WHITE && Right == WHITE && Rightest == BLACK)  // 1 0 0 0 1
      || (Leftest == BLACK && Left == WHITE && Center == WHITE && Right == BLACK && Rightest == WHITE)  // 1 0 0 1 0
      || (Leftest == BLACK && Left == WHITE && Center == BLACK && Right == WHITE && Rightest == WHITE)  // 1 0 1 0 0
      || (Leftest == BLACK && Left == WHITE && Center == BLACK && Right == WHITE && Rightest == BLACK)  // 1 0 1 0 1 //// consult with sarmad if this should be in counter
      || (Leftest == BLACK && Left == BLACK && Center == WHITE && Right == BLACK && Rightest == BLACK)  // 1 1 0 1 1 //// consult with sarmad
      || (Leftest == BLACK && Left == BLACK && Center == WHITE && Right == BLACK && Rightest == WHITE)  // 1 1 0 1 0
      || (Leftest == WHITE && Left == BLACK && Center == BLACK && Right == BLACK && Rightest == WHITE))
    == 1) {
    S_forward = 1;
    green.setCursor(13, 0);
    green.print("FF");
  } else if (
    // check for counter
    (
      // (Leftest == BLACK && Left == BLACK && Center == WHITE && Right == WHITE && Rightest == BLACK)    // 0 1 1 0 1   ////////////??????????????????????
      // // || (Leftest == WHITE && Left == BLACK && Center == BLACK && Right == BLACK && Rightest == WHITE)    // 0 1 1 1 0  ///iT'S FINE, OBVIOUS CASE FOR 45 DEGREE
      /* ||*/ (Leftest == WHITE && Left == BLACK && Center == BLACK && Right == BLACK && Rightest == BLACK)  // 0 1 1 1 1
      // || (Leftest == BLACK && Left == WHITE && Center == BLACK && Right == BLACK && Rightest == WHITE)    // 1 0 1 1 0 ////////////////????????????????
      // || (Leftest == BLACK && Left == WHITE && Center == BLACK && Right == BLACK && Rightest == BLACK)    // 1 0 1 1 1
      // || (Leftest == BLACK && Left == BLACK && Center == BLACK && Right == WHITE && Rightest == BLACK)    // 1 1 1 0 1
      || (Leftest == BLACK && Left == BLACK && Center == BLACK && Right == BLACK && Rightest == WHITE)  // 1 1 1 1 0
      || (Leftest == BLACK && Left == BLACK && Center == BLACK && Right == BLACK && Rightest == BLACK)  // 1 1 1 1 1
      )
    == 1) {
    S_counter = 1;
    S_forward = 1;

    green.setCursor(13, 0);
    green.print("FC");

  } else if (
    //check for right
    ((Leftest == WHITE && Left == WHITE && Center == WHITE && Right == BLACK && Rightest == WHITE)     // 0 0 0 1 0
      || (Leftest == WHITE && Left == WHITE && Center == BLACK && Right == BLACK && Rightest == WHITE) // 0 0 1 1 0
     
     )
    == 1) {
    S_right = 1;

    green.setCursor(13, 0);
    green.print("R ");
  } else if (
    //check for rightest
    ((Leftest == WHITE && Left == WHITE && Center == WHITE && Right == WHITE && Rightest == BLACK)     // 0 0 0 0 1
    
    || (Leftest == WHITE && Left == WHITE && Center == BLACK && Right == BLACK && Rightest == BLACK)  // 0 0 1 1 1
    || (Leftest == WHITE && Left == WHITE && Center == WHITE && Right == BLACK && Rightest == BLACK) // 0 0 0 1 1
    || (Leftest == BLACK && Left == WHITE && Center == WHITE && Right == BLACK && Rightest == BLACK) // 1 0 0 1 1
     )
    == 1) {
    S_righter = 1;

    green.setCursor(13, 0);
    green.print("RR");
  } else if (
    //check for left
    ((Leftest == WHITE && Left == BLACK && Center == WHITE && Right == WHITE && Rightest == WHITE)     // 0 1 0 0 0
     || (Leftest == WHITE && Left == BLACK && Center == BLACK && Right == WHITE && Rightest == WHITE)  // 0 1 1 0 0
     

     )
    == 1) {
    S_left = 1;

    green.setCursor(13, 0);
    green.print("L ");
  } else if (
    //check for leftest
    ((Leftest == BLACK && Left == WHITE && Center == WHITE && Right == WHITE && Rightest == WHITE)     // 1 0 0 0 0
     || (Leftest == BLACK && Left == BLACK && Center == WHITE && Right == WHITE && Rightest == WHITE)  // 1 1 0 0 0
     || (Leftest == BLACK && Left == BLACK && Center == WHITE && Right == WHITE && Rightest == BLACK)  // 1 1 0 0 1 
     || (Leftest == BLACK && Left == BLACK && Center == BLACK && Right == WHITE && Rightest == WHITE)  // 1 1 1 0 0
     )
    == 1) {
    S_lefter = 1;

    green.setCursor(13, 0);
    green.print("LL");
  }
}


void E_brake_light() {
  // for (int i = 50; i= 0; i--) {
  // for(int i = 50; i >= 0; i=i-10){
  //   analogWrite(RIGHT_PWM, i);
  //   analogWrite(LEFT_PWM, i);
  // }
  analogWrite(RIGHT_PWM, 0);
  analogWrite(LEFT_PWM, 0);
  // delay(1);
  // }
  
 // delay(50);
  digitalWrite(RIGHT_DIR, HIGH);
  digitalWrite(brake_R, LOW);
  // delay(30);
  digitalWrite(LEFT_DIR, HIGH);
  digitalWrite(brake_L, LOW);
  //  delay(30);
  delay(stop_delay_light);
  ramp_flag = 1;
}


void E_brake() {
  // for (int i = 50; i= 0; i--) {
  // for(int i = 50; i >= 0; i=i-10){
  //   analogWrite(RIGHT_PWM, i);
  //   analogWrite(LEFT_PWM, i);
  // }
  analogWrite(RIGHT_PWM, 0);
  analogWrite(LEFT_PWM, 0);
  // delay(1);
  // }
  
  // delay(50);
  digitalWrite(RIGHT_DIR, HIGH);
  digitalWrite(brake_R, LOW);
  // delay(30);
  digitalWrite(LEFT_DIR, HIGH);
  digitalWrite(brake_L, LOW);
  // delay(30);
  delay(stop_delay);
  ramp_flag = 1;
}

void E_brake_Turn() {

  digitalWrite(RIGHT_DIR, LOW);
  digitalWrite(LEFT_DIR, LOW);
  digitalWrite(brake_R, HIGH);
  digitalWrite(brake_L, HIGH);
  analogWrite(RIGHT_PWM, 0);
  analogWrite(LEFT_PWM, 0);

  delay(short_turn_delay);
}
void end() {
  LEFT_SERVO_MID;
  RIGHT_SERVO_MID;
  // sense();
  // line_follow();
  // backwards(50);
  // delay(170);
  // E_brake();
  

  if (side == 1) {
    left_turn(200, 275);
  } else {
    right_turn(200, 275);
  }
  E_brake();
  delay(100);

   while (back_limit != 0) {
    back_limit = digitalRead(BACK_LIMIT_SWITCH);
    backwards(90);//70//40
  }
  pwm_zero();
  BACK_SERVO_OPEN;
  delay(2000); //2000
  // BACK_SERVO_CLOSE;

    //   forward(50);
    //   delay(400);
    //   E_brake();
    // //delay(200);

    // if (side == 1) {
    //     left_turn(200, 220);
    //   } else {
    //     right_turn(200, 210);
    //   }
    // forward(50);


    if (side == 1) {
      forward(5,250);
      } else {
      forward(250,5);  //to change    
      }  
  delay(500);
  analogWrite(RIGHT_PWM, 0);
  analogWrite(LEFT_PWM, 0);
  delay(1000);
  // RIGHT_SERVO_IN;
  // delay(500);
  // LEFT_SERVO_IN;
  while (1) {
  }
}

void mechtest() {
  test = digitalRead(Debug_Button);
  while (test != 0) {
    test = digitalRead(Debug_Button);
    val = analogRead(potpin);           // reads the value of the potentiometer (value between 0 and 1023)
    val = map(val, 0, 1023, 0, 180);    // scale it for use with the servo (value between 0 and 180)
                                        // sets the servo position according to the scaled value
    val1 = analogRead(potpin1);         // reads the value of the potentiometer (value between 0 and 1023)
    val1 = map(val1, 0, 1023, 0, 180);  // scale it for use with the servo (value between 0 and 180)
    leftservo.write(val1);
    Serial.println(val);
    Serial.println(val1);  // sets the servo position according to the scaled value



    delay(15);
  }
}

// void sensestop()
// {
//     State = digitalRead(onSwitch);

//     Leftest = !digitalRead(LEFTEST);
//     Left = !digitalRead(LEFTER);
//     Center = !digitalRead(MIDDLE);
//     Right = !digitalRead(RIGHTER);
//     Rightest = !digitalRead(RIGHTEST);

//     side_sen = digitalRead(Side_sense);

//     bLeftest = digitalRead(B_LEFTEST);
//     bLeft = digitalRead(B_LEFTER);
//     bCenter = digitalRead(B_CENTER);
//     bRight = digitalRead(B_RIGHTER);
//     bRightest = digitalRead(B_RIGHTEST);
//     side = digitalRead(Side_switch);

//     // POT_1 = (float)analogRead(POT1) / 4;
//     // POT_2 = ((float)analogRead(POT2)) / 4;
//     // POT_3 = (float)analogRead(POT3) / 4;
//     // POT_1 = 50;
//     // POT_2 = 40;
//     // POT_3 = 40;
//     // POT_1 = (float)analogRead(POT1) / 4;
//     // POT_2 = ((float)analogRead(POT2)) / 4;
//     // POT_3 = (float)analogRead(POT3) / 4;


//     Serial.print("POT1:");
//     Serial.print(POT_1);
//     Serial.print(" ");
//     Serial.print("POT2:");
//     Serial.print(POT_2);
//     Serial.print(" ");
//     Serial.print("POT3:");
//     Serial.println(POT_3);

//     //front_distance = Sharp_Front.getDistance();

//    // gyrosense();
//     //BASE = POT_1;

//     // BASE = POT_1;
//     // //balancer = POT_2/(float)(255);
//     // TURN = 255*(POT_2/(float)(255));
//     // EDGE_TURN = 255*(POT_3/(float)(255));

//     BASE = 60;
//     TURN = 40;
//     EDGE_TURN = 40;


//     statecheckStop();

// }

// void statecheckStop()
// {
//     //S_counter = 0;
//     S_forward = 0;
//     S_left    = 0;
//     S_lefter  = 0;
//     S_right   = 0;
//     S_righter = 0;

//     if ( // check for forward ref my notes.
//         (
//           (Leftest == WHITE && Left == WHITE && Center == WHITE && Right == WHITE && Rightest == WHITE)    // 0 0 0 0 0
//             ||
//             (Leftest == WHITE && Left == WHITE && Center == BLACK && Right == WHITE && Rightest == WHITE) // 0 0 1 0 0
//             || (Leftest == WHITE && Left == WHITE && Center == BLACK && Right == WHITE && Rightest == BLACK) // 0 0 1 0 1
//             || (Leftest == WHITE && Left == BLACK && Center == WHITE && Right == WHITE && Rightest == BLACK) // 0 1 0 0 1
//             || (Leftest == WHITE && Left == BLACK && Center == WHITE && Right == BLACK && Rightest == WHITE) // 0 1 0 1 0
//             || (Leftest == WHITE && Left == BLACK && Center == WHITE && Right == BLACK && Rightest == BLACK) // 0 1 0 1 1
//             || (Leftest == BLACK && Left == WHITE && Center == WHITE && Right == WHITE && Rightest == BLACK) // 1 0 0 0 1
//             || (Leftest == BLACK && Left == WHITE && Center == WHITE && Right == BLACK && Rightest == WHITE) // 1 0 0 1 0
//             || (Leftest == BLACK && Left == WHITE && Center == BLACK && Right == WHITE && Rightest == WHITE) // 1 0 1 0 0
//             || (Leftest == BLACK && Left == WHITE && Center == BLACK && Right == WHITE && Rightest == BLACK) // 1 0 1 0 1 //// consult with sarmad if this should be in counter
//             || (Leftest == BLACK && Left == BLACK && Center == WHITE && Right == BLACK && Rightest == BLACK) // 1 1 0 1 1 //// consult with sarmad
//             || (Leftest == BLACK && Left == BLACK && Center == WHITE && Right == BLACK && Rightest == WHITE)
//             || (Leftest == WHITE && Left == BLACK && Center == BLACK && Right == BLACK && Rightest == WHITE)// 1 1 0 1 0
//             ) == 1) {
//                 S_forward = 1;
//                 green.setCursor(13,0);
//                 green.print("FF");
//     }else if (
//          // check for counter
//         (
//           (Leftest == BLACK && Left == BLACK && Center == WHITE && Right == WHITE && Rightest == BLACK)    // 0 1 1 0 1  ////////????????
//           // || (Leftest == WHITE && Left == BLACK && Center == BLACK && Right == BLACK && Rightest == WHITE)    // 0 1 1 1 0
//           ||
//           (Leftest == WHITE && Left == BLACK && Center == BLACK && Right == BLACK && Rightest == BLACK)    // 0 1 1 1 1    //////////////???????????
//           || (Leftest == BLACK && Left == WHITE && Center == BLACK && Right == BLACK && Rightest == WHITE)    // 1 0 1 1 0
//           || (Leftest == BLACK && Left == WHITE && Center == BLACK && Right == BLACK && Rightest == BLACK)    // 1 0 1 1 1
//           || (Leftest == BLACK && Left == BLACK && Center == BLACK && Right == WHITE && Rightest == BLACK)    // 1 1 1 0 1
//           || (Leftest == BLACK && Left == BLACK && Center == BLACK && Right == BLACK && Rightest == WHITE)    // 1 1 1 1 0
//           || (Leftest == BLACK && Left == BLACK && Center == BLACK && Right == BLACK && Rightest == BLACK)    // 1 1 1 1 1
//         ) == 1){
//         //S_counter = 1;
//         S_forward = 1;

//                 green.setCursor(13,0);
//                 green.print("FC");

//     }else if (
//         //check for right
//         (  (Leftest == WHITE && Left == WHITE && Center == WHITE && Right == BLACK && Rightest == WHITE)    // 0 0 0 1 0
//         || (Leftest == WHITE && Left == WHITE && Center == BLACK && Right == BLACK && Rightest == WHITE)    // 0 0 1 1 0
//         || (Leftest == WHITE && Left == WHITE && Center == BLACK && Right == BLACK && Rightest == BLACK)    // 0 0 1 1 1
//         ) ==1) {
//         S_right = 1;

//                 green.setCursor(13,0);
//                 green.print("R ");
//     }else if (
//         //check for rightest
//         (  (Leftest == WHITE && Left == WHITE && Center == WHITE && Right == WHITE && Rightest == BLACK)    // 0 0 0 0 1
//         || (Leftest == WHITE && Left == WHITE && Center == WHITE && Right == BLACK && Rightest == BLACK)    // 0 0 0 1 1
//         || (Leftest == BLACK && Left == WHITE && Center == WHITE && Right == BLACK && Rightest == BLACK)    // 1 0 0 1 1
//         ) == 1){
//         S_righter = 1;

//                 green.setCursor(13,0);
//                 green.print("RR");
//     }else if (
//         //check for left
//         (  (Leftest == WHITE && Left == BLACK && Center == WHITE && Right == WHITE && Rightest == WHITE)    // 0 1 0 0 0
//         || (Leftest == WHITE && Left == BLACK && Center == BLACK && Right == WHITE && Rightest == WHITE)    // 0 1 1 0 0
//         || (Leftest == BLACK && Left == BLACK && Center == BLACK && Right == WHITE && Rightest == WHITE)    // 1 1 1 0 0
//         ) == 1){
//             S_left = 1;

//                 green.setCursor(13,0);
//                 green.print("L ");
//     }else if (
//         //check for leftest
//         (  (Leftest == BLACK && Left == WHITE && Center == WHITE && Right == WHITE && Rightest == WHITE)    // 1 0 0 0 0
//         || (Leftest == BLACK && Left == BLACK && Center == WHITE && Right == WHITE && Rightest == WHITE)    // 1 1 0 0 0
//         || (Leftest == BLACK && Left == BLACK && Center == WHITE && Right == WHITE && Rightest == BLACK)    // 1 1 0 0 1
//         ) == 1){
//             S_lefter = 1;

//                 green.setCursor(13,0);
//                 green.print("LL");
//     }

// }

//Wait till debug button is pushed
void debug_button() {
  test = digitalRead(Debug_Button);
  while (test != 0) {
    test = digitalRead(Debug_Button);
  }
}

int process_green_value(int S2, int S3, int OUT){
  digitalWrite(S2, HIGH);
  digitalWrite(S3, HIGH);
  for (int i = 0; i<7; i++) {
  int pulse_length = pulseIn(OUT, LOW);
  delay(PROCESS_TIME);
  }
  int pulse_length = pulseIn(OUT, LOW);
  return pulse_length;
}

int process_red_value(int S2, int S3, int OUT){
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
  for (int i = 0; i<7; i++) {
  int pulse_length = pulseIn(OUT, LOW);
  delay(PROCESS_TIME);
  }
  int pulse_length = pulseIn(OUT, LOW);
  return pulse_length;
}
void simpleBrakeF(){
analogWrite(RIGHT_PWM, 0);
  analogWrite(LEFT_PWM, 0);
 // delay(50);
  digitalWrite(RIGHT_DIR, LOW);
  digitalWrite(LEFT_DIR, LOW);
  digitalWrite(brake_R, LOW);
  digitalWrite(brake_L, LOW);
  delay(stop_delay);
}
void pwm_zero(){
  analogWrite(RIGHT_PWM, 20);
  analogWrite(LEFT_PWM, 20);
}
