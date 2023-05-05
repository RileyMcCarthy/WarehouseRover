// Ultrasonics must use an interrupt pin each (2 total)
// Use new motion planning

//From top motors plug in with red at left side
// Motors go 1-4
#include <Arduino.h>

#include <SPI.h>
#include <MFRC522.h>

#define RST_PIN 44  // Configurable, see typical pin layout above
#define SS_PIN 49   // Configurable, see typical pin layout above

MFRC522 mfrc522(SS_PIN, RST_PIN);  // Create MFRC522 instance

#define MOTOR1_A 6
#define MOTOR1_B 24
#define ENCODER1_A 21
#define ENCODER1_B 29

#define MOTOR2_A 2
#define MOTOR2_B 26
#define ENCODER2_A 20
#define ENCODER2_B 33

#define MOTOR3_A 7
#define MOTOR3_B 40
#define ENCODER3_A 19
#define ENCODER3_B 37

#define MOTOR4_A 46
#define MOTOR4_B 48
#define ENCODER4_A 18
#define ENCODER4_B 14

#define MOTOR5_A 3
#define MOTOR5_B 34

#define MOTOR6_A 5
#define MOTOR6_B 4

#define trigPin1 47
#define echoPin1 45

#define trigPin3 45
#define echoPin3 43

#define Kp 20
#define Ki 0.01
#define Kd 0
#define T_PID_MS 0

#define MAX_ACCELERATION 300
#define MAX_VELOCITY 300

#define MOVE_ERROR 10

#define ROTATE_FACTOR 3.1

typedef struct motionPeriods_s {
  double x_goal;
  double x0;
  double v0;
  double v_max;
  double a_max;
  double T1;
  double T2;
  double T3;
  double a_acc;
  double a_dec;
  double v;
} MotionPeriod;

typedef struct setPoint_s {
  double t;
  double x;
  double v;
  double a;
} SetPoint;

typedef struct motor_s {
  bool reverse;
  int encoderA, encoderB;
  int motorA, motorB;
  long encoder;
  long encoderVelocity;
  unsigned long encoderLastTick;
  unsigned long encoderPeriod;
  long setpoint;
  long startTime;
  MotionPeriod periods;
  SetPoint motionSetpoint;
  int dir;
  long p[2], i, d;
} Motor;

typedef struct ultrasonic_s {
  int trig;
  int echo;
} Ultrasonic;

typedef enum moveType_e {
  MOVE_X,
  MOVE_Y,
  MOVE_DIAGONALLEFT,
  MOVE_DIAGONALRIGHT,
  SPIN_PLZ,
  MOVE_LIFT,  // +1 for up, -1 for down
  MOVE_GRAB,  // +1 for grab, -1 for release
} MoveType;

typedef struct Move_s {
  MoveType type;
  int delta;
  int delayms;
  bool ultrasonicCheck;
} Move;

typedef struct MoveSet_s {
  Move moves[30];
  int num_moves;
  int id;
} MoveSet;

Motor motor1;
Motor motor2;
Motor motor3;
Motor motor4;
Ultrasonic *sensor1;
Ultrasonic *sensor2;
Ultrasonic *sensor3;

#define BIN_COUNT 1

MoveSet bin_moves[BIN_COUNT];

double ultrasonic_read1(){
  pinMode(trigPin1,OUTPUT);
  digitalWrite(trigPin1, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(1);
  digitalWrite(trigPin1, LOW);
 
  // Read the signal from the sensor: a HIGH pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(echoPin1, INPUT);
  int duration = pulseIn(echoPin1, HIGH);
  Serial.print("readd");
  Serial.println(duration/5.8);
  return duration/5.80;
}

// Equation to determine position
double position1(double t, double xi, double vi, double a) {
  return (xi + vi * t + 0.5 * a * pow(t, 2));
}

double velocity(double t, double vi, double a) {
  return (vi + a * t);
}

void create_empty_setpoint(SetPoint *setpoint) {
  setpoint->t = 0;
  setpoint->x = 0;
  setpoint->v = 0;
  setpoint->a = 0;
}

void compute_period(MotionPeriod *periods, double x_goal, double x0, double v0, double v_max, double a_max) {
  double x_stop = fabs((-1 * v0 + sqrt(fabs(pow(v0, 2) - 4 * (-0.5 * a_max) * x0)) / (2 * (-0.5 * a_max))));

  int d;
  if (x_goal > x_stop) {
    d = 1;
  } else {
    d = -1;
  }
  int xGoalSign = 0;
  if (x_goal < 0) {
    xGoalSign = -1;
  } else if (x_goal > 0) {
    xGoalSign = 1;
  }
  int x0Sign = 0;
  if (x0 < 0) {
    x0Sign = -1;
  } else if (x0 > 0) {
    x0Sign = 1;
  }
  if (x0Sign == -1 && xGoalSign == 0) {
    d *= -1;
  }
  if (x0Sign == xGoalSign) {
    if (fabs(x0) > fabs(x_goal)) {
      Serial.println("Changing direction");
      d *= -1;
    }
  }
  double v = d * v_max;
  double a_acc = d * a_max;
  double a_dec = -1 * d * a_max;
  double T1 = fabs((v - v0) / a_acc);
  double T3 = fabs(v / a_dec);

  double X1 = position1(T1, 0, v0, a_acc);
  double X3 = position1(T3, 0, v, a_dec);

  double T2 = (x_goal - x0 - X1 - X3) / v;

  if (T2 < 0) {
    T2 = 0;
    v = d * sqrt(fabs(d * a_max * (x_goal - x0) + 0.5 * pow(v0, 2)));
    T1 = fabs((v - v0) / a_acc);
    T3 = fabs(v / a_dec);
  }
  periods->x_goal = x_goal;
  periods->x0 = x0;
  periods->v0 = v0;
  periods->v_max = v_max;
  periods->a_max = a_max;
  periods->T1 = T1;
  periods->T2 = T2;
  periods->T3 = T3;
  periods->a_acc = a_acc;
  periods->a_dec = a_dec;
  periods->v = v;
}

void compute_setpoint(SetPoint *setpoint, double t, MotionPeriod *periods) {
  double x1 = position1(periods->T1, periods->x0, periods->v0, periods->a_acc);
  double v2 = periods->v;
  double x2 = position1(periods->T2, x1, v2, 0);
  if (t <= 0) {
    setpoint->x = position1(0, periods->x0, periods->v0, 0);
    setpoint->v = velocity(0, periods->v0, 0);
    setpoint->a = 0;
  } else if (t < periods->T1) {
    setpoint->x = position1(t, periods->x0, periods->v0, periods->a_acc);
    setpoint->v = velocity(t, periods->v0, periods->a_acc);
    setpoint->a = periods->a_acc;
  } else if (t < periods->T1 + periods->T2) {
    setpoint->x = position1(t - periods->T1, x1, v2, 0);
    setpoint->v = velocity(t - periods->T1, v2, 0);
    setpoint->a = 0;
  } else if (t < periods->T1 + periods->T2 + periods->T3) {
    setpoint->x = position1(t - (periods->T1 + periods->T2), x2, v2, periods->a_dec);
    setpoint->v = velocity(t - (periods->T1 + periods->T2), v2, periods->a_dec);
    setpoint->a = periods->a_dec;
  } else {
    setpoint->x = periods->x_goal;
    setpoint->v = 0;
    setpoint->a = 0;
  }
}

double mm_to_ticks(int mm) {
  //140ticks/rev / 80mm/rev * mm
  return mm * (140.0 / (3.14159 * 80.0));
}

double ticks_to_mm(int ticks) {
  return ((double)ticks) / (140.0 / (3.14159 * 80.0));
}

void inc_a1() {
  int increment = 1;
  if (!motor1.reverse) {
    increment = -1;
  }
  if (digitalRead(motor1.encoderB)) {  //B->A
    motor1.encoder -= increment;
  } else {
    motor1.encoder += increment;
  }
}

void inc_a2() {
  int increment = 1;
  if (!motor2.reverse) {
    increment = -1;
  }
  if (digitalRead(motor2.encoderB)) {  //B->A
    motor2.encoder -= increment;
  } else {
    motor2.encoder += increment;
  }
}

void inc_a3() {
  int increment = 1;
  if (motor3.reverse) {
    increment = -1;
  }
  if (digitalRead(motor3.encoderB)) {  //B->A
    motor3.encoder -= increment;
  } else {
    motor3.encoder += increment;
  }
}

void inc_a4() {
  int increment = 1;
  if (motor4.reverse) {
    increment = -1;
  }
  if (digitalRead(motor4.encoderB)) {  //B->A
    motor4.encoder -= increment;
  } else {
    motor4.encoder += increment;
  }
}

void setup_motor(Motor *motor, int motorA, int motorB, int encoderA, int encoderB, bool reverse, int id, void (*func)()) {
  motor->encoder = 0;
  motor->encoderLastTick = 0;
  motor->encoderPeriod = 0;
  motor->startTime = 0;
  motor->setpoint = 0;
  motor->reverse = reverse;
  motor->p[0] = 0;
  motor->p[1] = 0;
  motor->i = 0;
  motor->d = 0;
  motor->motorA = motorA;
  motor->motorB = motorB;
  motor->encoderA = encoderA;
  motor->encoderB = encoderB;
  motor->periods.x_goal = 0;
  motor->periods.x0 = 0;
  motor->periods.v0 = 0;
  motor->periods.v_max = 0;
  motor->periods.a_max = 0;
  motor->periods.T1 = 0;
  motor->periods.T2 = 0;
  motor->periods.T3 = 0;
  motor->periods.a_acc = 0;
  motor->periods.a_dec = 0;
  motor->periods.v = 0;
  motor->motionSetpoint.t = 0;
  motor->motionSetpoint.x = 0;
  motor->motionSetpoint.v = 0;
  motor->motionSetpoint.a = 0;
  motor->dir = 0;
  pinMode(motor->encoderA, INPUT);
  pinMode(motor->encoderB, INPUT);
  pinMode(motor->motorA, OUTPUT);
  pinMode(motor->motorB, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(motor->encoderA), func, RISING);
}

void move_relative(Motor *motor, int setpoint) {
  double currentmm = ticks_to_mm(motor->encoder);
  motor->setpoint += setpoint;
  compute_period(&(motor->periods), motor->setpoint, currentmm, 0, MAX_VELOCITY, MAX_ACCELERATION);
  /*  Serial.print("Currentmm:");
  Serial.println(currentmm);
  Serial.print("t1:");
  Serial.println(motor->periods.T1);
  Serial.print("t2:");
  Serial.println(motor->periods.T2);
  Serial.print("t3:");
  Serial.println(motor->periods.T3);
  Serial.print("Currentmm:");
  Serial.println(motor->periods.T3);*/
  motor->startTime = millis();
}


void motor_pid_update(Motor *motor) {
  //PID Variables
  long currentTime = millis();
  long deltaTime = currentTime - motor->startTime;
  double velocity = 0;
  compute_setpoint(&(motor->motionSetpoint), deltaTime / 1000.0, &(motor->periods));
  int limitedSetpoint = (int)mm_to_ticks(motor->motionSetpoint.x);


  //Update PID values
  motor->p[1] = motor->p[0];
  motor->p[0] = limitedSetpoint - motor->encoder;
  motor->i += motor->p[0];
  motor->d = (motor->p[0] - motor->p[1]);

  //Update motor voltage from setpoint
  int voltage = motor->p[0] * Kp + motor->i * Ki + motor->d * Kd;
  if (motor->reverse) {
    voltage = voltage * -1;
  }

  int v_abs = fabs(voltage);
  if (v_abs > 255)
    v_abs = 255;
  if (voltage > 0) {  //Move CW
    analogWrite(motor->motorA, 255 - v_abs);
    digitalWrite(motor->motorB, HIGH);
  } else {  //Move CCW
    digitalWrite(motor->motorB, LOW);
    analogWrite(motor->motorA, v_abs);
  }
}


long nextMove;

void setup() {

  Serial.begin(9600);

  SPI.begin();                        // Init SPI bus
  mfrc522.PCD_Init();                 // Init MFRC522
  delay(4);                           // Optional delay. Some board do need more time after init to be ready, see Readme
  mfrc522.PCD_DumpVersionToSerial();  // Show details of PCD - MFRC522 Card Reader details
  Serial.println(F("Scan PICC to see UID, SAK, type, and data blocks..."));
  delay(1000);

  nextMove = 0;

  setup_motor(&motor1, MOTOR1_A, MOTOR1_B, ENCODER1_A, ENCODER1_B, false, 1, inc_a1);
  setup_motor(&motor2, MOTOR2_A, MOTOR2_B, ENCODER2_A, ENCODER2_B, true, 2, inc_a2);
  setup_motor(&motor3, MOTOR3_A, MOTOR3_B, ENCODER3_A, ENCODER3_B, false, 3, inc_a3);
  setup_motor(&motor4, MOTOR4_A, MOTOR4_B, ENCODER4_A, ENCODER4_B, true, 4, inc_a4);

  //pinMode(trigPin1,OUTPUT);
  //pinMode(echoPin1,INPUT);

  bin_moves[0].id = 62;  // Box #4

  //STANDARD MOVE START
  bin_moves[0].moves[0].type = MOVE_GRAB;
  bin_moves[0].moves[0].delta = 1;
  bin_moves[0].moves[0].delayms = 500;
  bin_moves[0].moves[0].ultrasonicCheck = false;

  bin_moves[0].moves[1].type = MOVE_LIFT;
  bin_moves[0].moves[1].delta = 1;
  bin_moves[0].moves[1].delayms = 300;
  bin_moves[0].moves[1].ultrasonicCheck = false;

  bin_moves[0].moves[2].type = MOVE_X;
  bin_moves[0].moves[2].delta = -914;
  bin_moves[0].moves[2].delayms = 5000;
  bin_moves[0].moves[2].ultrasonicCheck = false;

  bin_moves[0].moves[3].type = SPIN_PLZ;
  bin_moves[0].moves[3].delta = -90;
  bin_moves[0].moves[3].delayms = 5000;
  bin_moves[0].moves[3].ultrasonicCheck = false;

  bin_moves[0].moves[4].type = MOVE_X;
  bin_moves[0].moves[4].delta = 3048;
  bin_moves[0].moves[4].delayms = 5000;
  bin_moves[0].moves[4].ultrasonicCheck = false;
  //STANDARD MOVE END

  bin_moves[0].moves[5].type = MOVE_Y;
  bin_moves[0].moves[5].delta = -394;
  bin_moves[0].moves[5].delayms = 5000;
  bin_moves[0].moves[5].ultrasonicCheck = false;

  //STANDARD MOVE START
  bin_moves[0].moves[6].type = MOVE_X;
  bin_moves[0].moves[6].delta = 600;
  bin_moves[0].moves[6].delayms = 5000;
  bin_moves[0].moves[6].ultrasonicCheck = false;

  bin_moves[0].moves[7].type = MOVE_LIFT;
  bin_moves[0].moves[7].delta = -1;
  bin_moves[0].moves[7].delayms = 300;
  bin_moves[0].moves[7].ultrasonicCheck = false;

  bin_moves[0].moves[8].type = MOVE_GRAB;
  bin_moves[0].moves[8].delta = -1;
  bin_moves[0].moves[8].delayms = 500;
  bin_moves[0].moves[8].ultrasonicCheck = false;

  bin_moves[0].moves[9].type = MOVE_LIFT;
  bin_moves[0].moves[9].delta = -1;
  bin_moves[0].moves[9].delayms = 100;
  bin_moves[0].moves[9].ultrasonicCheck = false;

  bin_moves[0].moves[10].type = MOVE_X;
  bin_moves[0].moves[10].delta = -600;
  bin_moves[0].moves[10].delayms = 5000;
  bin_moves[0].moves[10].ultrasonicCheck = false;
  //STANDARD MOVE END

  bin_moves[0].moves[11].type = MOVE_Y;
  bin_moves[0].moves[11].delta = 394;
  bin_moves[0].moves[11].delayms = 5000;
  bin_moves[0].moves[11].ultrasonicCheck = false;

  //STANDARD MOVE START
  bin_moves[0].moves[12].type = SPIN_PLZ;
  bin_moves[0].moves[12].delta = -180;
  bin_moves[0].moves[12].delayms = 5000;
  bin_moves[0].moves[12].ultrasonicCheck = false;

  bin_moves[0].moves[13].type = MOVE_X;
  bin_moves[0].moves[13].delta = 3048;
  bin_moves[0].moves[13].delayms = 5000;
  bin_moves[0].moves[13].ultrasonicCheck = false;

  bin_moves[0].moves[14].type = SPIN_PLZ;
  bin_moves[0].moves[14].delta = 90;
  bin_moves[0].moves[14].delayms = 5000;
  bin_moves[0].moves[14].ultrasonicCheck = false;

  bin_moves[0].moves[15].type = MOVE_X;
  bin_moves[0].moves[15].delta = 914;
  bin_moves[0].moves[15].delayms = 5000;
  bin_moves[0].moves[15].ultrasonicCheck = false;
  int index = 16;
  bin_moves[0].moves[index].type = MOVE_Y;
  bin_moves[0].moves[index].delta = 250;
  bin_moves[0].moves[index].delayms = 300;
  bin_moves[0].moves[index].ultrasonicCheck = false;
  index++;

  bin_moves[0].moves[index].type = MOVE_Y;
  bin_moves[0].moves[index].delta = -190;
  bin_moves[0].moves[index].delayms = 300;
  bin_moves[0].moves[index].ultrasonicCheck = false;
  index++;

  bin_moves[0].moves[index].type = MOVE_X;
  bin_moves[0].moves[index].delta = 250;
  bin_moves[0].moves[index].delayms = 300;
  bin_moves[0].moves[index].ultrasonicCheck = true;
  index++;
  bin_moves[0].num_moves = index;
  //MOVE END, SHOULD WAIT FOR RFID READ FOR NEXT MOVESET


  //new package
  /*memcpy(&(bin_moves[1]),bin_moves[0],sizeof(bin_moves[0]));
  bin_moves[1].id = 63;
  bin_moves[1].moves[5].delta = -394; //new movement
  bin_moves[1].moves[11].delta = 394; //new movement*/

}

typedef enum stages_e {
  HOME,
  PACKAGE_WAIT,
  GRAB_PACKAGE,
  RFID_READ,
  MOVE_TO_BIN
} Stages;

void execute_move(Move *move) {

  switch (move->type) {
    case MOVE_X:
      if (move->ultrasonicCheck)
      {
        double distance = ultrasonic_read1();
        double difference = distance-move->delta;
        Serial.print("diff");
        Serial.println(difference);
        move_relative(&motor1, difference);
        move_relative(&motor2, difference);
        move_relative(&motor3, difference);
        move_relative(&motor4, difference);
        break;
      }
      move_relative(&motor1, move->delta);
      move_relative(&motor2, move->delta);
      move_relative(&motor3, move->delta);
      move_relative(&motor4, move->delta);
      break;
    case MOVE_Y:
      move_relative(&motor1, -1 * move->delta);
      move_relative(&motor2, move->delta);
      move_relative(&motor3, -1 * move->delta);
      move_relative(&motor4, move->delta);
      break;
    case MOVE_DIAGONALRIGHT:
      move_relative(&motor1, move->delta);
      move_relative(&motor2, 1);
      move_relative(&motor3, move->delta);
      move_relative(&motor4, 1);
      break;
    case MOVE_DIAGONALLEFT:
      move_relative(&motor1, 1);
      move_relative(&motor2, move->delta);
      move_relative(&motor3, 1);
      move_relative(&motor4, move->delta);
      break;
    case SPIN_PLZ:
      move_relative(&motor1, -1 * move->delta * ROTATE_FACTOR);
      move_relative(&motor2, move->delta * ROTATE_FACTOR);
      move_relative(&motor3, move->delta * ROTATE_FACTOR);
      move_relative(&motor4, -1 * move->delta * ROTATE_FACTOR);
      break;
    case MOVE_LIFT:
      if (move->delta > 0) {
        analogWrite(MOTOR6_A, 255);
        digitalWrite(MOTOR6_B, LOW);
        delay(move->delayms);
        analogWrite(MOTOR6_A, 0);
        digitalWrite(MOTOR6_B, 0);
      } else {
        analogWrite(MOTOR6_A, 0);
        digitalWrite(MOTOR6_B, HIGH);
        delay(move->delayms);
        analogWrite(MOTOR6_A, 0);
        digitalWrite(MOTOR6_B, 0);
      }
      delay(1000);
      break;
    case MOVE_GRAB:

      if (move->delta > 0) {
        analogWrite(MOTOR5_A, 255);
        digitalWrite(MOTOR5_B, LOW);
        delay(move->delayms);
        analogWrite(MOTOR5_A, 0);
        digitalWrite(MOTOR5_B, 0);
      } else {
        analogWrite(MOTOR5_A, 0);
        digitalWrite(MOTOR5_B, HIGH);
        delay(move->delayms);
        analogWrite(MOTOR5_A, 0);
        digitalWrite(MOTOR5_B, 0);
      }
      delay(1000);
      break;
  }
}

MoveSet *find_moveset_by_id(int id) {
  for (int i = 0; i < BIN_COUNT; i++) {
    if (bin_moves[i].id == id) {
      Serial.print("Found moveset with id:");
      Serial.println(bin_moves[i].id);
      return &(bin_moves[i]);
    }
  }
  return NULL;  // no moveset found
}

int read_rfid_id() {
  // Reset the loop if no new card present on the sensor/reader. This saves the entire process when idle.
  Serial.println("Checking for new card");
  if (!mfrc522.PICC_IsNewCardPresent()) {
    //Serial.println("no new card");
    return -1;
  }
  //Serial.println("new card present");
  // Select one of the cards
  if (!mfrc522.PICC_ReadCardSerial()) {
    // Serial.println("Cannot read card");
    return -1;
  }
  Serial.print("UID:");
  Serial.println(mfrc522.uid.uidByte[2]);

  // Dump debug info about the card; PICC_HaltA() is automatically called
  mfrc522.PICC_DumpToSerial(&(mfrc522.uid));
  return mfrc522.uid.uidByte[2];
}
int currentpackage = 0;
void loop() {

  // Move can be X,Y,DiAG_LEFT,DIAG_RIGHT,ROTATE
  // X,Y can be ultrasonic moves that determine distance sensors should read for position
  // MoveSet is a sequence of moves that should be executed

  // Cycle through stages
  // Start
  // Stage 0 -> Home robot (read ultrasonic sensors for reference, then move by offset to home position)
  // Stage 1 ->  wait for package (read ultrasonic sensor)
  // Package detected
  // Stage 2 -> move forward, pick up package, move backwards to home
  // Stage 3 -> Read package RFID sensor id
  // IF FAILURE: id cannot be read, return package and wait for package adjustment (Undo last moveset return to start)
  // Stage 4 -> Execute move set for package id (moves to bin, place package, undo moveset)
  // Return to start
  //double distance = ultrasonic_read1();
  //Serial.println(distance);
  //delay(1000);
  //return;
  int stage = HOME;
  long next_pid_update_ms = 0;

  int moveSetIndex = 0;
  bool new_move = false;

  MoveSet *currentMoveSet = NULL;
  MoveSet startingMoveSet;
  int index = 0;
  //open grippers, tilt down
  startingMoveSet.moves[index].type = MOVE_LIFT;
  startingMoveSet.moves[index].delta = -1;
  startingMoveSet.moves[index].delayms = 500;
  startingMoveSet.moves[index].ultrasonicCheck = false;
  index++;

  startingMoveSet.moves[index].type = MOVE_GRAB;
  startingMoveSet.moves[index].delta = 1;
  startingMoveSet.moves[index].delayms = 300;
  startingMoveSet.moves[index].ultrasonicCheck = false;
  index++;

  startingMoveSet.moves[index].type = MOVE_Y;
  startingMoveSet.moves[index].delta = 250;
  startingMoveSet.moves[index].delayms = 300;
  startingMoveSet.moves[index].ultrasonicCheck = false;
  index++;

  startingMoveSet.moves[index].type = MOVE_Y;
  startingMoveSet.moves[index].delta = -190;
  startingMoveSet.moves[index].delayms = 300;
  startingMoveSet.moves[index].ultrasonicCheck = false;
  index++;

  startingMoveSet.moves[index].type = MOVE_X;
  startingMoveSet.moves[index].delta = 250;
  startingMoveSet.moves[index].delayms = 300;
  startingMoveSet.moves[index].ultrasonicCheck = true;
  index++;

  startingMoveSet.moves[index].type = MOVE_GRAB;
  startingMoveSet.moves[index].delta = -1;
  startingMoveSet.moves[index].delayms = 300;
  startingMoveSet.moves[index].ultrasonicCheck = false;
  index++;

  /*startingMoveSet.moves[index].type = MOVE_DIAGONALLEFT;
  startingMoveSet.moves[index].delta = -300;
  startingMoveSet.moves[index].delayms = 300;
  startingMoveSet.moves[index].ultrasonicCheck = false;
  index++;*/
  startingMoveSet.num_moves = index;
  //waiting for rfid

  currentMoveSet = &startingMoveSet;
  int lastMoveIndex = -1;
  int lastCompletedMove = -1;
  while (true) {
    //update motor PID
    if (millis() > next_pid_update_ms) {
      //Serial.println("Updating PID");
      motor_pid_update(&motor1);
      motor_pid_update(&motor2);
      motor_pid_update(&motor3);
      motor_pid_update(&motor4);
      next_pid_update_ms = millis() + T_PID_MS;
    }
    if (millis() < nextMove) {
      continue;
    }
    if (currentMoveSet == NULL) {

      //Serial.println("Waiting for RFID");
      // wait for rfid to get next move set
      // Reset the loop if no new card present on the sensor/reader. This saves the entire process when idle.
      if 
      currentpackage++;
      currentMoveSet = find_moveset_by_id(currentpackage);
      moveSetIndex = 0;
      lastMoveIndex = -1;
    }

    if ((currentMoveSet != NULL) && (lastMoveIndex != moveSetIndex)) {
      Serial.print("Move started: ");
      Serial.println(moveSetIndex);
      execute_move(&(currentMoveSet->moves[moveSetIndex]));
      lastMoveIndex = moveSetIndex;
    }

    int motor1AbsError = fabs(motor1.setpoint - ticks_to_mm(motor1.encoder));
    int motor2AbsError = fabs(motor2.setpoint - ticks_to_mm(motor2.encoder));
    int motor3AbsError = fabs(motor3.setpoint - ticks_to_mm(motor3.encoder));
    int motor4AbsError = fabs(motor4.setpoint - ticks_to_mm(motor4.encoder));
    //Serial.println(motor1AbsError);
    if (motor1AbsError < MOVE_ERROR && motor2AbsError < MOVE_ERROR && motor3AbsError < MOVE_ERROR && motor4AbsError < MOVE_ERROR && currentMoveSet != NULL) {
      // Move complete
      Serial.println("Move complete");
      nextMove = millis() + 1000;
      moveSetIndex++;
      if (moveSetIndex >= currentMoveSet->num_moves) {
        Serial.println("MoveSet complete");
        moveSetIndex = 0;
        lastMoveIndex = -1;
        currentMoveSet = NULL;
      }
    }
  }

  //delay(10);
}