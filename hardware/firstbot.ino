#define ANALOG_READ    'a'
#define GET_BAUDRATE   'b'
#define PIN_MODE       'c'
#define DIGITAL_READ   'd'
#define READ_ENCODERS  'e'
#define MOTOR_SPEEDS   'm'
#define MOTOR_POSITIONS 'p'
#define MOTOR_RAW_PWM  'o'
//#define PING           'p'
#define RESET_ENCODERS 'r'
#define SERVO_WRITE    's'
#define SERVO_READ     't'
#define UPDATE_PID     'u'
#define DIGITAL_WRITE  'w'
#define ANALOG_WRITE   'x'
#define LEFT            0
#define RIGHT           1

#define BAUDRATE 57600
#define MAX_PWM 255

#define MOTOR_L_WCNT_PIN 2 // FS brown: Encoder 
#define MOTOR_R_WCNT_PIN 3 // FS brown: Encoder 

#define MOTOR_L_FR_PIN 7// FR white: Direction
#define MOTOR_R_FR_PIN 8// FR white: Direction

#define MOTOR_L_BREAK_PIN 9 // BREAK green: BREAK
#define MOTOR_R_BREAK_PIN 10 // BREAK green: BREAK

#define MOTOR_L_PWM_PIN 5 // PWM blue: SPEED
#define MOTOR_R_PWM_PIN 6 // PWM blue: SPEED

//int duration = 0; // the number of the pulses
int l_wheel_cnt =0;
int r_wheel_cnt =0;

// encoder
volatile long l_duration = 0;
volatile long r_duration = 0;

// encoder direction
volatile unsigned char l_reverse = 0;
volatile unsigned char r_reverse = 0;

long readEncoder(int i) {
  if (i == LEFT)
    return l_duration;
  else
    return r_duration;
}

void resetEncoder(int i) {
  if (i == LEFT)
    l_duration = 0;
  else
    r_duration = 0;
}

void resetEncoders() {
    resetEncoder(LEFT);
    resetEncoder(RIGHT);
}

// motor
void initMotorController() {
   digitalWrite(MOTOR_L_BREAK_PIN, LOW);
   digitalWrite(MOTOR_R_BREAK_PIN, LOW);
   digitalWrite(MOTOR_L_FR_PIN, LOW);
   digitalWrite(MOTOR_R_FR_PIN, HIGH); 
 }
 
 void setMotorSpeed(int i, int spd) {
 
   if (spd < 0)
   {
     spd = -spd;
     if (i == LEFT)
       l_reverse = 1;
     else
       r_reverse = 1;
   } else {
     if (i == LEFT)
       l_reverse = 0;
     else
       r_reverse = 0;
   }

   if (spd > 255)
      spd = 255;

   if (i == LEFT) { 
    spd == 0 ? digitalWrite(MOTOR_L_BREAK_PIN, HIGH): digitalWrite(MOTOR_L_BREAK_PIN, LOW);
    if      (l_reverse == 0) { digitalWrite(MOTOR_L_FR_PIN, LOW); analogWrite(MOTOR_L_PWM_PIN, map(spd, 0, 255, 255, 0)); }
    else if (l_reverse == 1) { digitalWrite(MOTOR_L_FR_PIN, HIGH); analogWrite(MOTOR_L_PWM_PIN, map(spd, 0, 255, 255, 0)); }
   }
   else /*if (i == RIGHT) //no need for condition*/ {
    spd == 0 ? digitalWrite(MOTOR_R_BREAK_PIN, HIGH): digitalWrite(MOTOR_R_BREAK_PIN, LOW);
    if      (r_reverse == 0) { digitalWrite(MOTOR_R_FR_PIN, HIGH); analogWrite(MOTOR_R_PWM_PIN, map(spd, 0, 255, 255, 0)); }
    else if (r_reverse == 1) { digitalWrite(MOTOR_R_FR_PIN, LOW); analogWrite(MOTOR_R_PWM_PIN, map(spd, 0, 255, 255, 0)); }
   }
 }
  
  void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    setMotorSpeed(LEFT, leftSpeed);
    setMotorSpeed(RIGHT, rightSpeed);
  }

// PID
#define PID_RATE           30     // Hz

/* Convert the rate into an interval */
const int PID_INTERVAL = 1000 / PID_RATE;
  
/* Track the next time we make a PID calculation */
unsigned long nextPID = PID_INTERVAL;

/* PID setpoint info For a Motor */
typedef struct {
  double TargetTicksPerFrame;    // target speed in ticks per frame
  double TargetTicks;    // target position in ticks per frame  
  double Encoder;                  // encoder count
  double PrevEnc;                  // last encoder count
  double PrevErr;                  // last error  
  double PrevInput;                // last input
  double ITerm;                    //integrated term
  double output;                    // last motor setting
}
SetPointInfo;

SetPointInfo leftPID, rightPID;

/* PID Parameters */
double Kp = 0.5;
double Kd = 0.2;
double Ki = 0.001;
int Ko = 1;

unsigned char isMoving = 0; // is the base in motion?
unsigned char isSpeedControl = 0;

void resetPID(){
   leftPID.TargetTicksPerFrame = 0.0;
   leftPID.Encoder = readEncoder(LEFT);
   leftPID.PrevEnc = leftPID.Encoder;
   leftPID.PrevErr = 0;
   leftPID.output = 0;
   leftPID.PrevInput = 0;
   leftPID.ITerm = 0;

   rightPID.TargetTicksPerFrame = 0.0;
   rightPID.Encoder = readEncoder(RIGHT);
   rightPID.PrevEnc = rightPID.Encoder;
   leftPID.PrevErr = 0;   
   rightPID.output = 0;
   rightPID.PrevInput = 0;
   rightPID.ITerm = 0;
}

void controlPosition(SetPointInfo * p);
void controlSpeed(SetPointInfo * p);

/* position */
void controlPosition(SetPointInfo * p) {
  double Kp = 0.2;
  double Kd = 0.003;
  double Ki = 0.0001;
  int Ko = 1;

  double Perror;
  double output;
  double input;

  Perror = p->TargetTicks - p->Encoder;
  output = Kp * Perror + Kd * (Perror - p->PrevErr) + p->ITerm;

  p->PrevErr = Perror;
  p->PrevEnc = p->Encoder;

  if (output >= MAX_PWM)
    output = MAX_PWM;
  else if (output <= -MAX_PWM)
    output = -MAX_PWM;
  else
    p->ITerm += Ki * Perror;

  p->output = output; // speed -> pwm
  p->PrevInput = input;
}

/* PID routine to compute the next motor commands */
void controlSpeed(SetPointInfo * p) {
  double Perror;
  double output;
  double input;

  input = (p->Encoder - p->PrevEnc);
//  input = p->PrevInput
//  p->PrevInput += input;
  Perror = p->TargetTicksPerFrame - input;

  output = Kp * Perror + Kd * (Perror - p->PrevErr) + p->ITerm;
//  output = (Kp * Perror - Kd * (input - p->PrevInput) + p->ITerm) / Ko;
  p->PrevErr = Perror;
  p->PrevEnc = p->Encoder;

  // speed
  if (output >= MAX_PWM)
    output = MAX_PWM;
  else if (output <= -MAX_PWM)
    output = -MAX_PWM;
  else
    p->ITerm += Ki * Perror;

  p->output += output; // accel -> speed -> pwm
  p->PrevInput = input;
}

/* Read the encoder values and call the PID routine */
void updateControl() {
  /* Read the encoders */
  leftPID.Encoder = readEncoder(LEFT);
  rightPID.Encoder = readEncoder(RIGHT);

  /* If we're not moving there is nothing more to do */
  if (!isMoving){
    if (leftPID.PrevInput != 0 || rightPID.PrevInput != 0) resetPID();
    return;
  }

  if (isSpeedControl){
    /* Compute PID update for each motor */
    controlSpeed(&leftPID);
    controlSpeed(&rightPID);
  } else {
    controlPosition(&leftPID);
    controlPosition(&rightPID);
  }

  // Serial.println("---");
  // Serial.print("PID.Error: ");
  // Serial.print(leftPID.PrevErr);
  // Serial.print(", ");
  // Serial.println(rightPID.PrevErr);

  // Serial.print("PID.output: ");
  // Serial.print(leftPID.output);
  // Serial.print(", ");
  // Serial.println(rightPID.output);

  // Serial.print("Speed: ");
  // Serial.print(leftPID.PrevInput);
  // Serial.print(" ticks/dt, ");
  // Serial.print(rightPID.PrevInput);
  // Serial.println(" ticks/dt");

  // Serial.print("Pulses: ");
  // Serial.print(l_duration);
  // Serial.print(", ");
  // Serial.println(r_duration);

  /* Set the motor speeds accordingly */
  setMotorSpeeds((int)leftPID.output, (int)rightPID.output);
}

// COMMAND
#define AUTO_STOP_INTERVAL 2000
//#define AUTO_STOP_INTERVAL 10000

long lastMotorCommand = AUTO_STOP_INTERVAL;

// A pair of varibles to help parse serial commands (thanks Fergs)
int arg = 0;
int index = 0; 

// Variable to hold the current single-character command
char cmd;

// Character arrays to hold the first and second arguments
char argv1[16];
char argv2[16];

// The arguments converted to integers
long arg1;
long arg2;

/* Clear the current command parameters */
void resetCommand() {
  cmd = NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  index = 0;
}

/* Run a command.  Commands are defined in commands.h */
int runCommand() {
  int i = 0;
  char *p = argv1;
  char *str;
  int pid_args[4];
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);
  
  switch(cmd) {
  case GET_BAUDRATE:
    Serial.println(BAUDRATE);
    break;
  case ANALOG_READ:
    Serial.println(analogRead(arg1));
    break;
  case DIGITAL_READ:
    Serial.println(digitalRead(arg1));
    break;
  case ANALOG_WRITE:
    analogWrite(arg1, arg2);
    Serial.println("OK"); 
    break;
  case DIGITAL_WRITE:
    if (arg2 == 0) digitalWrite(arg1, LOW);
    else if (arg2 == 1) digitalWrite(arg1, HIGH);
    Serial.println("OK"); 
    break;
  case PIN_MODE:
    if (arg2 == 0) pinMode(arg1, INPUT);
    else if (arg2 == 1) pinMode(arg1, OUTPUT);
    Serial.println("OK");
    break;
  case READ_ENCODERS:
    Serial.flush();
    Serial.print(readEncoder(LEFT));
    Serial.print(" ");
    Serial.println(readEncoder(RIGHT));
    break;
  case RESET_ENCODERS:
    resetEncoders();
    resetPID();
    Serial.println("OK");
    break;
  case MOTOR_SPEEDS:
    /* Reset the auto stop timer */
    lastMotorCommand = millis();
    if (arg1 == 0 && arg2 == 0) {
      setMotorSpeeds(0, 0);
      resetPID();
      isMoving = 0;
    }
    else isMoving = 1;
    isSpeedControl = 1;    
    leftPID.TargetTicksPerFrame = arg1;
    rightPID.TargetTicksPerFrame = arg2;
    Serial.println("OK"); 
    break;
  case MOTOR_POSITIONS:
    /* Reset the auto stop timer */
    lastMotorCommand = millis();
    if (arg1 == 0 && arg2 == 0) {
      setMotorSpeeds(0, 0);
      resetPID();
      isMoving = 0;
    }
    else isMoving = 1;
    isSpeedControl = 0;
    leftPID.TargetTicks = arg1;
    rightPID.TargetTicks = arg2;
    Serial.println("OK"); 
    break;    
  case MOTOR_RAW_PWM:
    /* Reset the auto stop timer */
    lastMotorCommand = millis();
    resetPID();
    isMoving = 0; // Sneaky way to temporarily disable the PID
    setMotorSpeeds(arg1, arg2);
    Serial.println("OK"); 
    break;
  case UPDATE_PID:
    while ((str = strtok_r(p, ":", &p)) != '\0') {
       //pid_args[i] = atoi(str);
       pid_args[i] = atof(str);
       i++;
    }
    Kp = pid_args[0];
    Kd = pid_args[1];
    Ki = pid_args[2];
    Ko = pid_args[3];
    Serial.println("OK");
    break;
   default:
    Serial.println("Invalid Command");
    break;
  }
}


void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  
  pinMode(MOTOR_L_BREAK_PIN, OUTPUT);
  pinMode(MOTOR_R_BREAK_PIN, OUTPUT);

  pinMode(MOTOR_L_FR_PIN, OUTPUT);
  pinMode(MOTOR_R_FR_PIN, OUTPUT);

  // digitalWrite(MOTOR_L_BREAK_PIN, LOW);
  // digitalWrite(MOTOR_R_BREAK_PIN, LOW);

  // setting the rising edge interrupt
  pinMode(MOTOR_L_WCNT_PIN, INPUT_PULLUP);
  pinMode(MOTOR_R_WCNT_PIN, INPUT_PULLUP);  
  attachInterrupt(digitalPinToInterrupt(MOTOR_L_WCNT_PIN), ISR_L_wheelCount, RISING);
  attachInterrupt(digitalPinToInterrupt(MOTOR_R_WCNT_PIN), ISR_R_wheelCount, RISING);  

  // Pins D5 and D6 - 4 kHz PWM
  TCCR0B = 0b00000010; // x8
  TCCR0A = 0b00000001; // phase correct
  initMotorController();

  Serial.begin(BAUDRATE);
}


// the loop function runs over and over again forever
void loop() {

  while (Serial.available() > 0) {
    // Read the next character
    char ch = Serial.read();

    // Terminate a command with a CR
    if (ch == 13) {
      if (arg == 1) argv1[index] = NULL;
      else if (arg == 2) argv2[index] = NULL;
      runCommand();
      resetCommand();
    }
    // Use spaces to delimit parts of the command
    else if (ch == ' ') {
      // Step through the arguments
      if (arg == 0) arg = 1;
      else if (arg == 1)  {
        argv1[index] = NULL;
        arg = 2;
        index = 0;
      }
      continue;
    }
    else {
      if (arg == 0) {
        // The first arg is the single-letter command
        cmd = ch;
      }
      else if (arg == 1) {
        // Subsequent arguments can be more than one character
        argv1[index] = ch;
        index++;
      }
      else if (arg == 2) {
        argv2[index] = ch;
        index++;
      }
    }
  }
 
  if (millis() > nextPID) {
    updateControl();
    nextPID += PID_INTERVAL;
  }
  
  // Check to see if we have exceeded the auto-stop interval
  if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) {;
    setMotorSpeeds(0, 0);
    isMoving = 0;
  }
}

// interrupt function for risig edge (opt --> RISING, FALLING, CHANGE, LOW)
// checking the one pulse of encoder motor (xiaomi)
void ISR_L_wheelCount()
{
  if (l_reverse == 0)
    l_duration = l_duration + 1;
  else
    l_duration = l_duration - 1;

  l_wheel_cnt = l_duration / 1200;
}

void ISR_R_wheelCount()
{
  if (r_reverse == 0)
    r_duration = r_duration + 1;
  else
    r_duration = r_duration - 1;

  r_wheel_cnt = r_duration / 1200;
}
