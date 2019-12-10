#include <AFMotor.h>
#include <Servo.h>
//#include <UltrasonicSensor.h>

#define ROBOT_MANUAL                        0
#define ROBOT_AUTOMATIC                     1

int currentMode = ROBOT_MANUAL;

#define CMD_FORWARD                         2
#define CMD_BACKWARD                        3
#define CMD_LEFT                            4
#define CMD_RIGHT                           5
#define CMD_STOP                            6

int cmdRobot;

#define ULTRASONIC_FRONT                    0
#define ULTRASONIC_FRONT_TRIG               13
#define ULTRASONIC_FRONT_ECHO               A1

#define ULTRASONIC_BACK                     1
#define ULTRASONIC_BACK_TRIG                2
#define ULTRASONIC_BACK_ECHO                A0

#define ULTRASONIC_MIN_DISTANCE             20
                              
//UltrasonicSensor ultrasonicSensors[] = { UltrasonicSensor(ULTRASONIC_FRONT_TRIG, ULTRASONIC_FRONT_ECHO), UltrasonicSensor(ULTRASONIC_BACK_TRIG, ULTRASONIC_BACK_ECHO) };

#define M1                                  0 // Rigth
#define M2                                  1 // Left
#define M3                                  2 // Rigth
#define M4                                  3 // Left
#define MOTOR_LENGTH                        4
#define MOTOR_MAX_SPEED                     255
#define MOTOR_LEFT_SPEED                    225
#define MOTOR_RIGHT_SPEED                   240

AF_DCMotor motors[] = { AF_DCMotor(1), AF_DCMotor(2), AF_DCMotor(3), AF_DCMotor(4) };
int currentFront = ULTRASONIC_FRONT;
int currentState = RELEASE;
int currentSpeed = 0;

#define SERVO 9 // Porta Digital 6 PWM
 
Servo s; // Variável Servo
int pos; // Posição Servo

void setup() 
{
  s.attach(SERVO);
  Serial.begin(9600);
  s.write(0); // Inicia motor posição zero
  
  /*Serial.begin(9600);

  for(int i = 0; i < MOTOR_LENGTH; i++)
  {
    motors[i].setSpeed(200);
    motors[i].run(RELEASE);
  }
  
  delay(5000);*/
  //startRobot();
}

void loop() 
{
  for(pos = 0; pos < 90; pos++)
  {
    s.write(pos);
    delay(15);
  }
  
  delay(1000);
  
  for(pos = 90; pos < 0; pos--)
  {
    s.write(pos);
    delay(15);
  }
  
  /*if(currentMode == ROBOT_AUTOMATIC)
    modeAutomatic();
  else if(currentMode == ROBOT_MANUAL)
    modeManual();*/
}

void modeAutomatic()
{
  /*boolean again = true;
  
  while(again)
  {
    if(Serial.available() > 0) 
    {
      cmdRobot = Serial.parseInt();
      
      if(cmdRobot == ROBOT_MANUAL)
      {
        currentMode = ROBOT_MANUAL;
        again = false;
      }
    }
    
    if(ultrasonicSensors[currentFront].getDistance() < ULTRASONIC_MIN_DISTANCE)
    {
      stopRobot();
      turnLeft();
    
      delay(1500);
    
      if(ultrasonicSensors[ULTRASONIC_FRONT].getDistance() > ULTRASONIC_MIN_DISTANCE)
        currentFront = ULTRASONIC_FRONT;
      else if(ultrasonicSensors[ULTRASONIC_BACK].getDistance() > ULTRASONIC_MIN_DISTANCE)
        currentFront = ULTRASONIC_BACK;
    }
    else
    {
      if(currentFront == ULTRASONIC_FRONT)
        moveToFront();
      else if (currentFront == ULTRASONIC_BACK)
        moveToBack();
    }
  }*/
}

void modeManual()
{
  /*boolean again = true;
  
  while(again)
  {
    if(Serial.available() > 0) 
    {
      cmdRobot = Serial.parseInt();
      
      if(cmdRobot == ROBOT_AUTOMATIC)
      {
        currentMode = ROBOT_AUTOMATIC;
        again = false;
      }
      else
      {
        switch(cmdRobot)
        {
          case CMD_FORWARD:
            if(ultrasonicSensors[ULTRASONIC_FRONT].getDistance() > ULTRASONIC_MIN_DISTANCE)
              moveToFront();
              break;
          case CMD_BACKWARD:
            if(ultrasonicSensors[ULTRASONIC_BACK].getDistance() > ULTRASONIC_MIN_DISTANCE)
              moveToBack();
              break;
          case CMD_LEFT:
              turnLeft();
              break;
          case CMD_RIGHT:
              turnRight();
              break;
          case CMD_STOP:
              stopRobot();
              break;
        }
      }
    }
  }*/
}

void startRobot()
{
  changeStateAndSpeedAllMotors(currentState, FORWARD, currentSpeed, MOTOR_MAX_SPEED);
}

void stopRobot()
{
  changeStateAndSpeedAllMotors(currentState, RELEASE, currentSpeed, 0);
}

void changeStateAndSpeedAllMotors(int currentState, int newState, int currentSpeed, int newSpeed)
{
  changeStateAllMotors(currentState, newState);
  changeSpeedAllMotors(newSpeed);
}

void changeStateAndSpeedMotor(int motorId, int currentState, int newState, int currentSpeed, int newSpeed)
{
  changeStateMotor(motorId, currentState, newState);
  changeSpeedMotor(motorId, currentSpeed, newSpeed);
}

void changeStateAllMotors(int currentState, int newState)
{
  for(int i = 0; i < MOTOR_LENGTH; i++)
  {
      changeStateMotor(i, currentState, newState);
  }
}

void changeStateMotor(int motorId, int currentState, int newState)
{
  if(motorId < MOTOR_LENGTH)
  {
    switch(newState)
    {
      case FORWARD:
        if(currentState == BACKWARD)
          changeSpeedMotor(motorId, currentSpeed, 0);
        
        motors[motorId].run(FORWARD);
        currentState = FORWARD;
        break;
      case BACKWARD:
        if(currentState == FORWARD)
          changeSpeedMotor(motorId, currentSpeed, 0);
        
        motors[motorId].run(BACKWARD);
        currentState = BACKWARD;
        break;
      case RELEASE:
        changeSpeedMotor(motorId, currentSpeed, 0);
        motors[motorId].run(RELEASE);
        currentState = RELEASE;
        break;
    }
  }
}

void changeSpeedAllMotors(int newSpeed)
{
  if(newSpeed < MOTOR_MAX_SPEED)
  {
    for(int i = 0; i < MOTOR_LENGTH; i++)
    {
      changeSpeedMotor(i, currentSpeed, newSpeed);
    }
  }
}

void changeSpeedMotor(int motorId, int currentSpeed, int newSpeed)
{
  if((motorId < MOTOR_LENGTH) && (newSpeed < MOTOR_MAX_SPEED))
  {
    if(currentSpeed < newSpeed)
    {
      for(int i = currentSpeed; i < newSpeed; i++)
      {
        motors[motorId].setSpeed(i);
        currentSpeed += i;
      }
    }
    else if(currentSpeed > newSpeed)
    {
      for(int i = currentSpeed; i >= newSpeed; i--)
      {
        motors[motorId].setSpeed(i);
        currentSpeed += i;
      }
    }
  }
}

void moveToFront()
{
  changeStateAndSpeedAllMotors(currentState, FORWARD, currentSpeed, MOTOR_MAX_SPEED);
}

void moveToBack()
{
  changeStateAndSpeedAllMotors(currentState, BACKWARD, currentSpeed, MOTOR_MAX_SPEED);
}

void turnRight()
{
  changeStateAndSpeedMotor(M1, currentState, BACKWARD, currentSpeed, MOTOR_LEFT_SPEED);
  changeStateAndSpeedMotor(M2, currentState, FORWARD, currentSpeed, MOTOR_RIGHT_SPEED);
  changeStateAndSpeedMotor(M3, currentState, BACKWARD, currentSpeed, MOTOR_LEFT_SPEED);
  changeStateAndSpeedMotor(M4, currentState, FORWARD, currentSpeed, MOTOR_RIGHT_SPEED);
}

void turnLeft()
{
  changeStateAndSpeedMotor(M1, currentState, FORWARD, currentSpeed, MOTOR_RIGHT_SPEED);
  changeStateAndSpeedMotor(M2, currentState, BACKWARD, currentSpeed, MOTOR_LEFT_SPEED);
  changeStateAndSpeedMotor(M3, currentState, FORWARD, currentSpeed, MOTOR_RIGHT_SPEED);
  changeStateAndSpeedMotor(M4, currentState, BACKWARD, currentSpeed, MOTOR_LEFT_SPEED);
}
