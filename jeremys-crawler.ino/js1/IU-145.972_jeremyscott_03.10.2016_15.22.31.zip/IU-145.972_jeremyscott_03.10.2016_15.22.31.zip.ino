#include <microBox.h> // usb interface

#include <Servo.h>    //to define and control servos
#include <FlexiTimer2.h>        //to set a timer to manage all servos
#include <EEPROM.h>   //to save errors of all servos
#include <SPI.h>    //nRF24L01 module need 1/3
#include <nRF24L01.h>         //nRF24L01 module need 2/3
#include <RF24.h>   //nRF24L01 module need 3/3

// microbox
char historyBuf[100];
char hostname[] = "ioBash";

PARAM_ENTRY Params[]=
{
  {"hostname", hostname, PARTYPE_STRING | PARTYPE_RW, sizeof(hostname), NULL, NULL, 0}, 
  {NULL, NULL}
};

// calibrator
static float EXIT = -10000;
static float SKIP = -20000;
static int UPPERBOUND = 0;
static int LOWERBOUND = 1;
static int MIDPOINT   = 2;

// movement
const int LEGS = 4;
const int JOINTS = 3;
const int COMMAND_DURATION = 3;
float SPEED = 60;
float target[LEGS][JOINTS];
float transit[LEGS][JOINTS];
float jointSpeed[LEGS][JOINTS];
float requestedValue[LEGS][JOINTS];

/* Servos --------------------------------------------------------------------*/
//define 12 servos for 4 legs
Servo servo[4][3];
//define servos' ports
const int servo_pin[4][3] = { 2, 3, 4, 5, 6, 7, 14, 15, 16, 17, 18, 19 };

void setup() {
  // micro box
  Serial.begin(115200);
  microbox.begin(&Params[0], hostname, true, historyBuf, 100);
  microbox.AddCommand("s", mbSetServo);
  microbox.AddCommand("c", mb_calibrate);
  microbox.AddCommand("reset", mbReset);
  microbox.AddCommand("do", mbDoCommand);
  microbox.AddCommand("set", mbSetCommand);
  microbox.AddCommand("status", mbStatusCommand);

  Serial.println("Robot starts initialization");

  initServos();
  Serial.println("Servos initialized");
  
  FlexiTimer2::set(20, servo_service);
  FlexiTimer2::start();
  Serial.println("Servo Service initialized");
 
  doCommandSit();
  
  Serial.println("Robot initialization Complete");
}


void initServos() {
  //initialize servos
  for (int leg = 0; leg < LEGS; leg++)
  {
    for (int joint = 0; joint < JOINTS; joint++)
    {
      servo[leg][joint].attach(servo_pin[leg][joint]);
      delay(100);
      transit[leg][joint] = servo[leg][joint].read();
    }
  }
}

void loop() {
  microbox.cmdParser();
}

void mbSetCommand(char **param, uint8_t parCnt) {
  String command = param[0];
  
  if (command == "speed") {
    SPEED = atof(param[1]);
  }
}

void mbStatusCommand(char **param, uint8_t parCnt) {
  Serial.println("Status");
  for (int leg = 0; leg < LEGS; leg++) {
    for (int joint = 0; joint < JOINTS; joint++) {
      Serial.print(String("leg: "));
      Serial.print(String(leg));
      Serial.print(" joint: ");
      Serial.print(String(joint));
      Serial.print(" servo: ");
      Serial.print(String(servo[leg][joint].read()));
      Serial.print(" target: ");
      Serial.print(String(getCurrentPosition(leg, joint)));
      Serial.print(" requestedValue: ");
      Serial.println(String(requestedValue[leg][joint]));
    }
  }
}

void mbDoCommand(char **param, uint8_t parCnt) {
  String command = param[0];

  unsigned long start = millis();
  
  if (command == "sit") {
    doCommandSit();
  } else if (command == "stand") {
    doCommandStand();
  } else if (command == "forward") {
    doCommandForward();
  } else if (command == "leg" || command == "all") {
    int d = 1;
    int minLeg, maxLeg;
    if(command == "leg") { 
      d = 0;
      int leg = atoi(param[1]);
      minLeg = leg;
      maxLeg = leg + 1;
    } else {
      minLeg = 0;
      maxLeg = LEGS;
    }

    int j0 = atof(param[2 - d]);
    int j1 = atof(param[3 - d]);
    int j2 = atof(param[4 - d]);

    for (int leg = minLeg; leg < maxLeg; leg++)
    {
      float command[4];
      makeCommand(command, 1000, j0, j1, j2);

      Serial.print(String("leg: "));
      Serial.print(String(leg));
      Serial.print(" j0: ");
      Serial.print(String(j0));
      Serial.print(" j1: ");
      Serial.print(String(j1));
      Serial.print(" j2: ");
      Serial.println(String(j2));
      
      setLeg(leg, command);
    }
  }
  unsigned long end = millis();

  Serial.print("Took ");
  Serial.println(String(end - start));
}

float* makeCommand(float* command, float durationMs, float j0, float j1, float j2) {
  command[0] = j0;
  command[1] = j1;
  command[2] = j2;
  command[COMMAND_DURATION] = durationMs;

  return command;
}

void doCommandSit() {
  float command[4];
  makeCommand(command, 1000, 100, 90, 50);
  setAllLegs(command);
  waitAllReached();
  makeCommand(command, 1000, 100, 70, 50 );
  setAllLegs(command);
  waitAllReached();
}

void doCommandStand() {
  float command[4];
  makeCommand(command, 1000, 100, 90, 50);
  setAllLegs(command);
  waitAllReached();
  makeCommand(command, 1000, 50, 50, 50);
  setAllLegs(command);
  waitAllReached();
}

void doCommandForward() {
  float command[4];
  float stdSpeed = 2000;

  // 0
  setLeg(0, makeCommand(command, stdSpeed, 100, 70, 50));
  setLeg(1, makeCommand(command, stdSpeed, 100, 70, 50));
  setLeg(2, makeCommand(command, stdSpeed, 100, 70, 0));
  setLeg(3, makeCommand(command, stdSpeed, 100, 70, 0));
  waitAllReached();

  // 1
  setLeg(0, makeCommand(command, stdSpeed, 100, 70, 50));
  setLeg(1, makeCommand(command, stdSpeed, 100, 70, 50));
  setLeg(2, makeCommand(command, stdSpeed, 100, 70, 50));
  setLeg(3, makeCommand(command, stdSpeed, 100, 70, 0));
  waitAllReached();

  // 2
  setLeg(0, makeCommand(command, stdSpeed, 100, 70, 0));
  setLeg(1, makeCommand(command, stdSpeed, 100, 70, 50));
  setLeg(2, makeCommand(command, stdSpeed, 100, 70, 50));
  setLeg(3, makeCommand(command, stdSpeed, 100, 70, 50));
  waitAllReached();

  // 3
  setLeg(0, makeCommand(command, stdSpeed, 100, 70, 0));
  setLeg(1, makeCommand(command, stdSpeed, 100, 70, 0));
  setLeg(2, makeCommand(command, stdSpeed, 100, 70, 50));
  setLeg(3, makeCommand(command, stdSpeed, 100, 70, 50));
  waitAllReached();
  
  // 4
  setLeg(0, makeCommand(command, stdSpeed, 100, 70, 50));
  setLeg(1, makeCommand(command, stdSpeed, 100, 70, 0));
  setLeg(2, makeCommand(command, stdSpeed, 100, 70, 50));
  setLeg(3, makeCommand(command, stdSpeed, 100, 70, 50));
  waitAllReached();

  // 5
  setLeg(0, makeCommand(command, stdSpeed, 100, 70, 50));
  setLeg(1, makeCommand(command, stdSpeed, 100, 70, 50));
  setLeg(2, makeCommand(command, stdSpeed, 100, 70, 0));
  setLeg(3, makeCommand(command, stdSpeed, 100, 70, 100));
  waitAllReached();

  // 0
  setLeg(0, makeCommand(command, stdSpeed, 100, 70, 50));
  setLeg(1, makeCommand(command, stdSpeed, 100, 70, 50));
  setLeg(2, makeCommand(command, stdSpeed, 100, 70, 0));
  setLeg(3, makeCommand(command, stdSpeed, 100, 70, 0));
  waitAllReached();

}

float getCurrentPosition(int leg, int joint) {
  return transit[leg][joint];
}

float getJointTarget(int leg, int joint) {
  return target[leg][joint];
}

void mbSetServo(char **param, uint8_t parCnt) 
{
  if (parCnt == 1) {
    String command = param[0];
    if (command == "i") {
      initServos();  
    } else if (command == "s") {
      for (int leg = 0; leg < LEGS; leg++)
      {
        for (int joint = 0; joint < JOINTS; joint++)
        {
          Serial.print(String("leg: "));
          Serial.print(String(leg));
          Serial.print(" joint: ");
          Serial.print(String(joint));
          Serial.print(" value: ");
          Serial.println(String(servo[leg][joint].read()));
        }
      }
    }
    
  } else {
    uint8_t leg = atoi(param[0]);
    uint8_t joint = atoi(param[1]);
    float pos = atof(param[2]); 
    setLegJoint(leg, joint, pos, 1000);
  }
}

void setLegs(float **command) {
  for (int leg = 0; leg < LEGS; leg++) {
    setLeg(leg, command[leg]);
  }
  waitAllReached();
}

void setAllLegs(float *command) {
  for (int leg = 0; leg < LEGS; leg++) {
    setLeg(leg, command);
  }
}

void setLeg(int leg, float *command) {
   for (int joint = 0; joint < JOINTS; joint++) {
      setLegJoint(leg, joint, command[joint], command[COMMAND_DURATION]);
   }
}

void setLegJoint(int leg, int joint, float value, float durationMs) {
  requestedValue[leg][joint] = value;
  
  float upperBound =  readEeprom(leg, joint, UPPERBOUND);
  float lowerBound =  readEeprom(leg, joint, LOWERBOUND);
  float midpoint =    readEeprom(leg, joint, MIDPOINT);
  float percent =     (upperBound - lowerBound) / 100;

  // legs 0 and 3 are inversed
  if (leg == 0 || leg == 3) {
    value = 100 - value;
  } 

  float delta = value - 50;
  float targetPostition = midpoint + (delta * percent);
  
  targetPostition = constrain(targetPostition, lowerBound, upperBound);

  target[leg][joint] = targetPostition;

  float current = getCurrentPosition(leg, joint);
  float diff = current - targetPostition;
  float jSpeed = (abs(diff) / durationMs) * SPEED;
  
  jointSpeed[leg][joint] = max(abs(jSpeed), 0.01);

  Serial.print(String("leg: "));
  Serial.print(String(leg));
  Serial.print(" joint: ");
  Serial.print(String(joint));
  Serial.print(" target: ");
  Serial.print(String(targetPostition));
  Serial.print(" speed: ");
  Serial.println(String(jointSpeed[leg][joint]));
}

void waitAllReached() {
  for (int leg = 0; leg < LEGS; leg++) {
    waitReached(leg);
  }
}

void waitReached(int leg)
{
  float current;
  float target;
  while (1) {
    int reached = 0;
    for (int joint = 0; joint < JOINTS; joint++)
    {
      current = getCurrentPosition(leg, joint);
      target = getJointTarget(leg, joint);

      if(current != target) {
        /*Serial.print(String("WAITING leg: "));
        Serial.print(String(leg));
        Serial.print(" joint: ");
        Serial.print(String(joint));
        Serial.print(" target: ");
        Serial.print(String(target));
        Serial.print(" current: ");
        Serial.println(String(current));*/
      } else {
        /*Serial.print(String("REACHED leg: "));
        Serial.print(String(leg));
        Serial.print(" joint: ");
        Serial.print(String(joint));
        Serial.print(" target: ");
        Serial.print(String(target));
        Serial.print(" current: ");
        Serial.println(String(current));*/
        reached++;
      }
    }
    if (reached == JOINTS) {
      return;
    }
  }
}



float readEeprom(int leg, int joint, int field) {
   return EEPROM.read(leg * 9 + joint * 3 + field);
}

void consumeInput() {
  if(Serial.available()) {
    while(Serial.available()) Serial.read();
  }
}

void servo_service(void) {
  sei(); // enable interrupts

  float current;
  float jointTarget;
  float jSpeed;

  for (int leg = 0; leg < LEGS; leg++) {
    for (int joint = 0; joint < JOINTS; joint++) {
      current = getCurrentPosition(leg, joint);
      jointTarget = getJointTarget(leg, joint);
      jSpeed = jointSpeed[leg][joint];
      
      if (current != jointTarget) {

        if (current < jointTarget) {
          current = transit[leg][joint] = min(current + jSpeed, jointTarget);
        } else {
          current = transit[leg][joint] = max(current - jSpeed, jointTarget);
        }

        Serial.print(String("TRANSIT leg: "));
        Serial.print(String(leg));
        Serial.print(" joint: ");
        Serial.print(String(joint));
        Serial.print(" target: ");
        Serial.print(String(jointTarget));
        Serial.print(" current: ");
        Serial.println(String(current));
        
        servo[leg][joint].write(current);
      } 
    }
  }
}


//////////
/// 
/// CALIBRATION
///
//////////
void mbReset(char **param, uint8_t parCnt) {
  for (int leg = 0; leg < LEGS; leg++)
  {
    for (int joint = 0; joint < JOINTS; joint++)
    {
      doSave(leg, joint, 180, 0, 90);
    }
  }
}

void mbShowCalibration(char **param, uint8_t parCnt) {
  Serial.println("Saved Calibration");
  
  for (int leg = 0; leg < LEGS; leg++)
  {
    for (int joint = 0; joint < JOINTS; joint++)
    {
        Serial.print(String("leg: "));
        Serial.print(String(leg));
        Serial.print(" joint: ");
        Serial.print(String(joint));
        Serial.print(" upper: ");
        Serial.print(readEeprom(leg, joint, UPPERBOUND));
        Serial.print(" lower: ");
        Serial.print(readEeprom(leg, joint, LOWERBOUND));
        Serial.print(" midpoint: ");
        Serial.println(readEeprom(leg, joint, MIDPOINT));
    }
  }
}

void mb_calibrate(char **param, uint8_t parCnt) 
{
  uint8_t leg = 0;
  uint8_t joint = 0; 
  if (parCnt == 0) {
    // lets do all legs! 
    for(; leg < LEGS; leg++) {
      for(joint = JOINTS; joint > 0; joint--) {
        float results[3]; 
        doCalibration(leg, joint, results);

        float upperBound = results[0];
        float lowerBound = results[1];
        float midpoint = results[2];

        if (upperBound == EXIT) {
          return;
        } else if (upperBound == SKIP) {
          continue;
        }

        save(leg, joint, upperBound, lowerBound, midpoint);
      }
    }
    
  } else {
    leg = atoi(param[0]);
    joint = atoi(param[1]);

    float results[3]; 
    doCalibration(leg, joint, results);
    float upperBound = results[0];
    float lowerBound = results[1];
    float midpoint = results[2];

    if (upperBound == EXIT || upperBound == SKIP) {
      return;
    }

    save(leg, joint, upperBound, lowerBound, midpoint);
  }
}

void save(uint8_t leg, uint8_t joint, float upperBound, float lowerBound, float midpoint) {
  Serial.println("Upper Bound = " + String(upperBound));
  Serial.println("Lower Bound = " + String(lowerBound));
  Serial.println("Mid point = " + String(midpoint));
  Serial.println("Save? y/n");
  while(true) {
    int inputChar = Serial.read();
    if (inputChar == 'y') {
      consumeInput();
      doSave(leg, joint, upperBound, lowerBound, midpoint);
      break;
    } else if (inputChar == 'n') {
      consumeInput();
      break;
    }
  }
}

void doSave(int leg, int joint, float upperBound, float lowerBound, float midpoint) {
  writeEeprom(leg, joint, UPPERBOUND, upperBound);
  writeEeprom(leg, joint, LOWERBOUND, lowerBound);
  writeEeprom(leg, joint, MIDPOINT, midpoint);
}

void doCalibration(uint8_t leg, uint8_t joint, float *results) {
  float upperBound = getServoBound(leg, joint, 1);
  if (upperBound == EXIT) {
    return {EXIT, 0.0};
  } else if (upperBound == SKIP) {
    return {SKIP, 0.0};
  }
  
  float lowerBound = getServoBound(leg, joint, -1);
  if (lowerBound == EXIT) {
    return {EXIT, 0.0};
  } else if (lowerBound == SKIP) {
    return {SKIP, 0.0};
  }

  float midpoint = getServoMidpoint(leg, joint, 1, lowerBound + ((upperBound - lowerBound) / 2));
  if (midpoint == EXIT) {
    return {EXIT, 0.0};
  } else if (midpoint == SKIP) {
    return {SKIP, 0.0};
  }

  results[0] = upperBound;
  results[1] = lowerBound;
  results[2] = midpoint;
}

float getServoBound(uint8_t leg, uint8_t joint, float delta) {
  float calibrate_pos = 90.0;
  boolean stop = false;
  Serial.print("Beginning calibration of ");
  Serial.print(String(leg));
  Serial.print(" joint ");
  Serial.print(String(joint));
  Serial.println(". Any key to mark");

  consumeInput();
  
  servo[leg][joint].write(calibrate_pos);
  while(!stop) {
    delay(100);
    if (calibrate_pos > 0 && calibrate_pos < 180) {
      calibrate_pos = calibrate_pos + delta;
    }
    
    Serial.println(String(calibrate_pos));
    servo[leg][joint].write(calibrate_pos);

    if(Serial.available()) {
      stop = true;
      consumeInput();
    }
  } 

  // step back some
  Serial.println("Use < >   . to save ! to stop ~ to skip");
  calibrate_pos = calibrate_pos + (delta * -5);
  servo[leg][joint].write(calibrate_pos);
  Serial.println(String(calibrate_pos));
  
  stop = false;
  while(!stop) {
    
    int inputChar;
    while(Serial.available()) {
      inputChar = Serial.read();
      if (inputChar == 60) {
        calibrate_pos = calibrate_pos + (delta * -1);
      } else if (inputChar == 62) {
        calibrate_pos = calibrate_pos + delta;
      } else if (inputChar == 46) {
        stop = true;
        consumeInput();
        break;
      } else if (inputChar == 33) {
        servo[leg][joint].write(90.0);
        consumeInput();
        return EXIT;
      } else if (inputChar == 126) {
        servo[leg][joint].write(90.0);
        consumeInput();
        return SKIP;
      }

      Serial.println(String(calibrate_pos));
      servo[leg][joint].write(calibrate_pos);
    }

    delay(15);
  }
  servo[leg][joint].write(90.0);
  return calibrate_pos;
}

float getServoMidpoint(uint8_t leg, uint8_t joint, float delta, float startPos) {
  boolean stop = false;
  float calibrate_pos = startPos;
  
  Serial.print("Beginning Midpoint calibration of ");
  Serial.print(String(leg));
  Serial.print(" joint ");
  Serial.print(String(joint));
  Serial.println(". Any key to mark");
  
  Serial.println("Use < >   . to save ! to stop ~ to skip");
  servo[leg][joint].write(calibrate_pos);
  Serial.println(String(calibrate_pos));

  consumeInput();
  stop = false;
  while(!stop) {
    
    int inputChar;
    while(Serial.available()) {
      inputChar = Serial.read();
      if (inputChar == '<') {
        calibrate_pos = calibrate_pos + (delta * -1);
      } else if (inputChar == '>') {
        calibrate_pos = calibrate_pos + delta;
      } else if (inputChar == '.') {
        stop = true;
        consumeInput();
        break;
      } else if (inputChar == '!') {
        servo[leg][joint].write(startPos);
        consumeInput();
        return EXIT;
      } else if (inputChar == '~') {
        servo[leg][joint].write(startPos);
        consumeInput();
        return SKIP;
      }

      Serial.println(String(calibrate_pos));
      servo[leg][joint].write(calibrate_pos);
    }

    delay(15);
  }
  servo[leg][joint].write(calibrate_pos);
  return calibrate_pos;
}

void writeEeprom(int leg, int joint, int field, float value) {
   EEPROM.write(leg * 9 + joint * 3 + field, value);
}


