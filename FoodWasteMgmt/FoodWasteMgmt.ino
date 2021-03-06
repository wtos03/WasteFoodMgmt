/*This program control the Waste Food Managemet Machine.
   It will control 2 DC Motors  and 1 Servo
      o  Motor1  control Food Crusher
      o  Motor2  control Stirring Rod
      o  Servo   control EM (Effective microorganisms)compost valve

   Process start from
      o Food crusher into small pieces
      o Add EM
      o Stir to mix EM with Waste Food.

    The Panel control  have 4 control Modes
      1.  Auto Mode mean start from Food crusher until Stir ingredient together.
      2.  Food crusher turn on/off  Food cruhser's motor only.
      3.  EM valve control to fill extra EM.
      4.  Stir mode turn on/off  Stir's motor only.



   Created by: Wichai Tossamartvorakul wichai.tos@gmail.com
   Date  16/8/20


*/


#include <Tlv493d.h>
#include <TLE94112-ino.hpp>
#include <TLE94112Motor-ino.hpp>
#include "SPI.h"

// Define constant use in program

//#define  DEBUG

#define  MAX_LED  4     // Knob control LED  4 
#define  TURNON_LED     LOW   // Depend on how you connect LED in this case LED's Negative pin connect to IO port
#define  TURNOFF_LED    HIGH
#define  ON    1
#define  OFF   0

// Port use for Display LED and Control Motor
#define  AUTO_LED           0
#define  MOTOR_CRUSH_LED    1
#define  START_STOP_SW      2
#define  SERVO_EM_LED       8
#define  MOTOR_STIR_LED     9
#define  WORKING_LED        6
#define  SERVO_PORT         5

//  For Motor Start/Stop (order in switch case
#define  MOTOR_CRUSH    1
#define  SERVO_EM       2
#define  MOTOR_STIR     3     


#define STIR_TIME         10000


 
// Knob control constant
#define  START_ANGLE    0.0
#define  NO_CHANGE  0
#define  KNOB_FORWARD   1
#define  KNOB_BACKWARD  -1
#define  KNOB_CLICK  2
#define  KNOB_NOACTION  0
#define  CLICK_AMOUNT 50
#define  FORWARD_AMOUNT   -40
#define  BACKWARD_AMOUNT  40
#define  SWITCH_PRESS 1

// Servo Control for MG996
#define MIN_ANGLE 0  // Minimum Servo angle
#define MAX_ANGLE 180  // Maximum servo angle
#define MIN_VALVE   60  // Want to move backward
#define MAX_VALVE   0
#define MIN_PULSE_WIDTH 540     // PulseWidth for on-pulse to write 0 degree angle on servo
#define MAX_PULSE_WIDTH 2000    // PulseWidth for on-pulse to write 180 degree angle on servo
#define MIN_DUTY_CYCLE 2
#define MAX_DUTY_CYCLE 25

//Global Variable
// MAP Table between LED Panel to Actual Port usage 
int ledPanel[MAX_LED] =  {AUTO_LED,MOTOR_CRUSH_LED,SERVO_EM_LED,MOTOR_STIR_LED};   
int switchStatus = OFF;    // ISP Keypress *****
int selectedLED = 0;


// Tlv493d Opject
Tlv493d Tlv493dMagnetic3DSensor = Tlv493d();

// Tle94112 Object
Tle94112Ino controller = Tle94112Ino();

// Tle94112Motor Objects
Tle94112Motor crusher_motor(controller);
Tle94112Motor stir_motor(controller);


void setup() {
  pinMode(MOTOR_CRUSH_LED, OUTPUT);
  pinMode(SERVO_EM_LED, OUTPUT);
  pinMode(MOTOR_STIR_LED, OUTPUT);
  pinMode(AUTO_LED, OUTPUT);
  pinMode(WORKING_LED, OUTPUT);
  pinMode(SERVO_PORT, OUTPUT);
  pinMode(START_STOP_SW,INPUT_PULLUP);
 // attachInterrupt(digitalPinToInterrupt(START_STOP_SW), startStopSW, FALLING);
  
  digitalWrite(WORKING_LED, TURNOFF_LED);
  showLED(AUTO_LED);  // WORKING_LED  Separate from other LED

  Serial.begin(115200);


  while (!Serial);
  Tlv493dMagnetic3DSensor.begin();
  setAnalogWriteFrequency (SERVO_PORT, 22); //Test these number get around 50 Hz 20 ms
  setAnalogWriteFrequency (WORKING_LED, 2); //Test these number get around 50 Hz 2 ms
  
  
  // Enable MotorController on all Shields and Motors
  // Note: Required to be done before starting to configure the motor
  // controller is set to default CS1 pin
  controller.begin();

  // Connect a motor to HB1/HB2 highside and HB3/HB4 lowside
  // With two combined halfbridges the motor can have up to 1.8 A
  // IMPORTANT connect PWM to Lowside as higside is active Free wheeling
  // Lowside give +  Use HB1-4  connector for Half bridge 
 // crusher_motor.initConnector(crusher_motor.HIGHSIDE, controller.TLE_NOPWM, controller.TLE_HB1, controller.TLE_HB2, controller.TLE_NOHB, controller.TLE_NOHB);
  crusher_motor.initConnector(crusher_motor.LOWSIDE,  controller.TLE_PWM1, controller.TLE_HB3, controller.TLE_HB4, controller.TLE_HB1, controller.TLE_HB2);
  stir_motor.initConnector(crusher_motor.HIGHSIDE, controller.TLE_NOPWM, controller.TLE_HB9, controller.TLE_HB10, controller.TLE_NOHB, controller.TLE_NOHB);
  stir_motor.initConnector(crusher_motor.LOWSIDE,  controller.TLE_PWM1,  controller.TLE_HB11, controller.TLE_HB12, controller.TLE_NOHB, controller.TLE_NOHB);

  // start the motor controller
  crusher_motor.begin();
  stir_motor.begin();
  servoWrite(MIN_VALVE);            // Reset servo Position
  Serial.println ("Start Program");
 

}

void loop() {
  int direction;
  int toggle = 0;
  delay(300);  // Control sentivity of the knob
  direction =  chkMovement();
  selectedLED = knobPosition(direction);
  Serial.print(" LED Position:  ");
  Serial.println(selectedLED);
// Check Start/Stop Key
  
  if (digitalRead(START_STOP_SW) == LOW)
  {
    chkStartStopKey(selectedLED);
 //   switchStatus = 0;    
  }
   
 
}

/*
 * Start Stop SW  ISR
 * Global  variable   status 
 */
void startStopSW(void)
{
  static int count = 0;
     count++;
     switchStatus = SWITCH_PRESS; 
     Serial.print ("Key Pressed");
     Serial.println (count);
  
}
   
   
/*
 * Show LED on the panel TURNON Selected LED and  TURNOFF all others LED.
 * This procedure will map LED's position to Port use in program.
 * INPUT   Position of LED on Panel
 */
void showLED( int led)
{
//TURN ALL OFF FIRST  FOR SIMPLE PROGRAMMING
  digitalWrite(MOTOR_CRUSH_LED, TURNOFF_LED);
  digitalWrite(SERVO_EM_LED, TURNOFF_LED);
  digitalWrite(MOTOR_STIR_LED, TURNOFF_LED);
  digitalWrite(AUTO_LED, TURNOFF_LED);
// TURN ON WHAT YOU WANT
  digitalWrite(ledPanel[led], TURNON_LED);   // MAP LED Panel to Acutal Port

}



/* Check movement of Knob by checking magnetic 3D sensor
    If different angle  >  20 mean move forward
    If different angle  < -20 mean move backward
    If magnetic field change > 20 mean knob had been click

*/

int chkMovement ()
{
  float static  prev_angle = START_ANGLE;
  float static  prev_amount = 0.0;
  float         current_angle = 0.0;
  float         diff_angle = 0.0;
  float         current_amount = 0.0;
  float         diff_amount = 0.0;
  int           direction = NO_CHANGE;
  Tlv493dMagnetic3DSensor.updateData();
  current_amount = Tlv493dMagnetic3DSensor.getAmount();
  diff_amount = current_amount - prev_amount;
  prev_amount = current_amount;
  current_angle = (Tlv493dMagnetic3DSensor.getAzimuth()) * 100.0; // Azimuth angle is very small need to scale up
  diff_angle =  current_angle - prev_angle;
  prev_angle = current_angle;
  if (diff_angle < FORWARD_AMOUNT)
    direction =  KNOB_FORWARD;
  else if (diff_angle > BACKWARD_AMOUNT)
    direction = KNOB_BACKWARD;
  else
    direction = KNOB_NOACTION;
  // if amount more than  20 then ignore any rotate movement sensor just report it's click
  if (diff_amount > CLICK_AMOUNT)
  {
    direction  = KNOB_CLICK ;  // Click
  }

#ifdef DEBUG
  Serial.print("Amount = ");
  Serial.print(current_amount);
  Serial.print(" mT; Azimuth angle (rad) = ");
  Serial.print(current_angle);
  Serial.print(" mT; Different angle (rad) = ");
  Serial.print(diff_angle);
  Serial.print("  Direction = ");
  Serial.println(direction);
#endif
  return direction;
}

/*
 * Check KNOB postion on the panel
 * INPUT  : Direction of movement ( FORWARD,BACKWARD, CLICK)
 * OUTPUT : Position of LED on Panel 0 -3
 */

int  knobPosition(int direction)
{
  int static  currentled = 0; 
  unsigned int static  toggle = 0;
  if (direction == KNOB_FORWARD )
  {
    currentled++;
    if (currentled >= MAX_LED)
      currentled = 0;
  }

  if (direction == KNOB_BACKWARD )
  {
    currentled--;
    if (currentled < 0)
      currentled = 0;
  }
  showLED (currentled);
  return currentled;

}

/* Check whether start/stop key pressed  if yes start or stop devices
 * 
 */
void chkStartStopKey (int led)
{
    // Check Switch Press
  static int toggle = 0;
    toggle = ~toggle;  // Beware toogle between 0 and 0xFFFF not 0 and 1
    if (toggle) // ON
    {
      if (led == AUTO_LED)  // AUTO PROGRAM Start
      {
            autoStartStop();
      }
      else
      {
          deviceStart(led); 
      }
    }
    else // OFF
    {
          deviceStop(led); // MAP Position of LED to Port
    }
    
   if (led == SERVO_EM)  // SERVO_EM is Push ON/Off  Not toggle state like other  
    {
        toggle = ~toggle;
    }
    Serial.println(toggle);
}  





/*
 * This procedure called when users click sw show at Auto position. It will start from motor crusher until
 * stop, open EM valve  and start stir motor.
 */
void autoStartStop (void)
{
  
  // Start crusher motor for XXX time or until stop sensor trigger
  int state  = 1;
  int endProgram = 0;
  int crushStart  = 0;
  unsigned int stirTime = 0;

  switchStatus = 0;
  do 
  {
     Serial.print (" State = ");
     Serial.println (state);
     switch (state)
     {
      case 0:
          state++;
          break;
      case 1:  // TURN ON MOTOR CRUSH
          if (crushStart == 0)
          {           
              deviceStart(MOTOR_CRUSH);
              crushStart = 1;
          }
          break;
      case 2:  
          if (stirTime == 0)
          {
              deviceStart(SERVO_EM);
              deviceStart(MOTOR_STIR);
          }
          
          if (stirTime < STIR_TIME)
          {
            stirTime++;
            Serial.println( stirTime);
            Serial.println(switchStatus);
          }
          else
          { 
            deviceStop (MOTOR_STIR);
            endProgram = 1;
            stirTime = 0;
          }
          break;
       }   // End switch
         
       if (digitalRead(START_STOP_SW) == LOW)  
       {
         if (state == 1) // Still at crusher motor and press start/stop go to next state
         {
           state++; // Next state
 //          delay (1000); // Key bounce protect 
 //          switchStatus = 0; // Clear key press status
         }
         else    // End program
         {
          endProgram = 1;  
          stirTime = 0;
          crushStart = 0;
          switchStatus = 0; // Clear Switch Press
          switch (state)
          {
            case 1:
              deviceStop (MOTOR_CRUSH);
              break;
            case 2:
              deviceStop (MOTOR_STIR);
              break;
          }
         }  
       }     
    
  } while (!endProgram);  

}


/*
   To start device  Crusher Motor, Stir Motor or Servo
   Can st   switchStatus = 0;
  art one device at a time ????
*/
void deviceStart (int device_no)
{
//   digitalWrite (device_no,TURNON_LED);     // TURN ON Devices' LED Make Stir motor not start ??
   showLED (device_no);
   analogWrite(WORKING_LED, 100);
   
  switch (device_no)
  {
    case MOTOR_CRUSH:
       delay (1000); // Prevent Motor to stop suddenly after start
       stir_motor.coast();
       crusher_motor.setSpeed(0);
       crusher_motor.rampSpeed(-255,10000);   // Use start Big motor cannot start ???
 //      crusher_motor.start (-255);
       break;
    case SERVO_EM:
      stir_motor.coast();
      crusher_motor.coast();
      emValve();  // No need delay because had been delay in emValve
      break;
    case  MOTOR_STIR:
      delay (1000); // Prevent Motor to stop suddenly after start
      crusher_motor.coast();
 //     stir_motor.setSpeed(0);
 //     stir_motor.rampSpeed(255,5000);   // Use start Big motor cannot start ???
      stir_motor.start(255);
      break;
  }

}


/*
   To stop device  Crusher Motor, Stir Motor or Servo
*/
void deviceStop (int device_no)
{
  analogWrite(WORKING_LED, 255);  
  switch (device_no)
  {
    case MOTOR_CRUSH:
      delay (1000); // Prevent Motor to stop suddenly after start
      crusher_motor.coast();
      break;
    case  SERVO_EM:
      break;
    case  MOTOR_STIR:
      delay (1000); // Prevent Motor to stop suddenly after start
      stir_motor.coast();
      break;

  }

}

/*
 * For EM Value just open and close
 */
void emValve()
{  
    servoWrite (MAX_VALVE);
    servoWrite (MIN_VALVE);
    //Turn off Working LED
    analogWrite(WORKING_LED, 255); 
}


void servoWrite (int angle)
{
  int duty_cycle;
  float pulse_width;
  int result;
  angle = constrain(angle, MIN_ANGLE, MAX_ANGLE); // Constraining the angle to avoid motor cluttering due to unusual pulses at higher angles
  //   pulse_width=map(angle,MIN_ANGLE,MAX_ANGLE,MIN_PULSE_WIDTH,MAX_PULSE_WIDTH);  // Boundaries to be calibrated by trial and error
  //   duty_cycle=map(pulse_width,MIN_PULSE_WIDTH,MAX_PULSE_WIDTH,MIN_DUTY_CYCLE,MAX_DUTY_CYCLE);  // Boundaries to be calibrated by trial and error
  duty_cycle = map(angle, MIN_ANGLE, MAX_ANGLE, MIN_DUTY_CYCLE, MAX_DUTY_CYCLE); // Boundaries to be calibrated by trial and error
 Serial.print ( "Duty Cycle =  ");
 Serial.println (duty_cycle);
  analogWrite(SERVO_PORT, duty_cycle);
  delay(2000);
  Serial.println ("Moved servo "); 
  analogWrite(SERVO_PORT,0); // Disable PWM Prevent servo hot
 

}

void servWrite(int angle)
{
    float delay_time;
    int i;
    for ( i = 0 ; i < 30 ; i++)
    {
      angle=constrain(angle,MIN_ANGLE,MAX_ANGLE);  // Constraining the angle to avoid motor cluttering due to unusual pulses at higher angles
      delay_time=map(angle,MIN_ANGLE,MAX_ANGLE,MIN_PULSE_WIDTH,MAX_PULSE_WIDTH);  // Boundaries to be calibrated by trial and error
      digitalWrite(SERVO_PORT,HIGH);
      delayUs(delay_time);
      digitalWrite(SERVO_PORT,LOW);
      Serial.println (angle);
      Serial.println (delay_time);
      delayUs((20000-delay_time));  // Because servo sg90 requires a total pulse of 20mS with proper duty cycle
    }
}

void delayUs(unsigned long uS)
{
  unsigned long time_now = micros();
      while(micros() < time_now + uS);
}
