/*This program control the Waste Food Managemet Machine. 
 * It will control 2 DC Motors  and 1 Servo 
 *    o  Motor1  control Food Crusher
 *    o  Motor2  control Stirring Rod
 *    o  Servo   control EM (Effective microorganisms)compost valve
 *   
 * Process start from   
 *    o Food crusher into small pieces
 *    o Add EM
 *    o Stir to mix EM with Waste Food.
 *    
 *  The Panel control  have 4 control Modes
 *    1.  Auto Mode mean start from Food crusher until Stir ingredient together.
 *    2.  Food crusher turn on/off  Food cruhser's motor only.
 *    3.  EM valve control to fill extra EM.
 *    4.  Stir mode turn on/off  Stir's motor only.
 *    
 * 
 * 
 * Created by: Wichai Tossamartvorakul wichai.tos@gmail.com
 * Date  16/8/20
 * 
 * 
 */


#include <Tlv493d.h>

// Define constant use in program


#define  MAX_LED  3     // Knob control LED  4 because start from 0,1,2,3
#define  TURNON_LED     LOW   // Depend on how you connect LED in this case LED's Negative pin connect to IO port
#define  TURNOFF_LED    HIGH

// Port use for Display LED and Control Motor
#define  AUTO_LED           0
#define  MOTOR_CRUNCH_LED   1
#define  MOTOR_EM_LED       2
#define  MOTOR_STIR_LED     3
#define  WORKING_LED        4
#define  SERVO_EM           5

// Knob control constant
#define  START_ANGLE    0.0
#define  NO_CHANGE  0
#define  KNOB_FORWARD   1
#define  KNOB_BACKWARD  -1
#define  KNOB_CLICK  2
#define  KNOB_NOACTION  0
#define  CLICK_AMOUNT 60
#define  FORWARD_AMOUNT 20
#define  BACKWARD_AMOUNT -20


// Servo Control for MG996
#define MIN_ANGLE 0  // Minimum Servo angle
#define MAX_ANGLE 180  // Maximum servo angle
#define MIN_PULSE_WIDTH 540   // 540 uS : PulseWidth for on-pulse to write 0 degree angle on servo
#define MAX_PULSE_WIDTH 2000    //20000 uS : PulseWidth for on-pulse to write 180 degree angle on servo
#define MIN_DUTY_CYCLE 10
#define MAX_DUTY_CYCLE 255


// Tlv493d Opject
Tlv493d Tlv493dMagnetic3DSensor = Tlv493d();

void setup() {
  pinMode(MOTOR_CRUNCH_LED, OUTPUT);
  pinMode(MOTOR_EM_LED, OUTPUT);
  pinMode(MOTOR_STIR_LED, OUTPUT);
  pinMode(AUTO_LED, OUTPUT);
  pinMode(WORKING_LED, OUTPUT);
  pinMode(SERVO_EM,OUTPUT);
  digitalWrite(MOTOR_CRUNCH_LED, TURNOFF_LED); 
  digitalWrite(MOTOR_EM_LED, TURNOFF_LED);   
  digitalWrite(MOTOR_STIR_LED, TURNOFF_LED);   
  digitalWrite(AUTO_LED, TURNON_LED);   
  digitalWrite(WORKING_LED, TURNOFF_LED);
 
  Serial.begin(115200);
  while(!Serial);
  Tlv493dMagnetic3DSensor.begin();
  servoWrite(0);    // Reset servo Position
}

void loop() {
  delay(300);
  int direction =  chkMovement();
   showLED(direction);

}

/* Check movement of Knob by checking magnetic 3D sensor 
 *  If different angle  >  20 mean move forward
 *  If different angle  < -20 mean move backward
 *  If magnetic field change > 20 mean knob had been click 
 * 
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
  current_angle = (Tlv493dMagnetic3DSensor.getAzimuth())* 100.0; // Azumuth angle is very small need to scale up
  diff_angle =  current_angle - prev_angle;
  prev_angle = current_angle;
  if (diff_angle > FORWARD_AMOUNT) 
      direction =  KNOB_FORWARD;
  else if (diff_angle < BACKWARD_AMOUNT)
          direction = KNOB_BACKWARD;
      else 
          direction = KNOB_NOACTION;
// if amount more than  20 then ignore any rotate movement sensor just report it's click
   if (diff_amount > CLICK_AMOUNT)
   {
        direction  = KNOB_CLICK ;  // Click      
   } 
//  Serial.print(" ; Polar angle (rad) = ");
//  Serial.println(Tlv493dMagnetic3DSensor.getPolar());

 /* 
  Serial.print("Amount = ");
  Serial.print(current_amount);
  Serial.print(" mT; Azimuth angle (rad) = ");
  Serial.print(current_angle);
  Serial.print(" mT; Different angle (rad) = ");
  Serial.print(diff_angle); 
  Serial.print("  Direction = ");
  Serial.println(direction);
*/  
  return direction;
}


void  showLED(int direction)
{
  int static  currentled = 0;
  int static  toggle = 0;
  if (direction == KNOB_FORWARD )  
  {
   digitalWrite(currentled, TURNOFF_LED );    
   currentled++;
   if (currentled > MAX_LED)
        currentled = 0;
   digitalWrite(currentled, TURNON_LED );   
  }

  if (direction == KNOB_BACKWARD )  
  {
   digitalWrite(currentled, TURNOFF_LED );    
   currentled--;
   if (currentled < 0)
   {
        currentled = 0;
   }     
   digitalWrite(currentled, TURNON_LED );    // turn the LED on
  }

  if (direction == KNOB_CLICK)
  {
    if(toggle)
    {
        digitalWrite (WORKING_LED,TURNON_LED);
        servoWrite (90);
    }
    else
    {
        digitalWrite(WORKING_LED, TURNOFF_LED);
        servoWrite(0);
    }
    toggle = ~toggle;
  }
//  Serial.print ("Current LED =  ");
//  Serial.println ( currentled);

}


void servoWrite (int angle)
{
    float duty, pulsewidth;
    angle=constrain(angle,MIN_ANGLE,MAX_ANGLE);  // Constraining the angle to avoid motor cluttering due to unusual pulses at higher angles
    pulsewidth =map(angle,MIN_ANGLE,MAX_ANGLE,MIN_PULSE_WIDTH,MAX_PULSE_WIDTH);
    duty=map(pulsewidth,MIN_PULSE_WIDTH,MAX_PULSE_WIDTH,MIN_DUTY_CYCLE,MAX_DUTY_CYCLE);  // Boundaries to be calibrated by trial and error
    analogWrite(SERVO_EM,duty);
    Serial.print ("PulseWidth =  ");
    Serial.print (pulsewidth);
    Serial.print ("  Duty =  ");
    Serial.println (duty);
}


void servWrite(int angle)
{
    float delay_time;
    int i;
    for ( i = 0 ; i < 20 ; i++)
    {
      angle=constrain(angle,MIN_ANGLE,MAX_ANGLE);  // Constraining the angle to avoid motor cluttering due to unusual pulses at higher angles
      delay_time=map(angle,MIN_ANGLE,MAX_ANGLE,MIN_PULSE_WIDTH,MAX_PULSE_WIDTH);  // Boundaries to be calibrated by trial and error
      digitalWrite(SERVO_EM,HIGH);
      delayUs(delay_time);
      digitalWrite(SERVO_EM,LOW);
      Serial.println (angle);
      Serial.println (delay_time);
      delayUs((20000-delay_time));  // Because servo MG996 requires a total pulse of 20mS with proper duty cycle
    }
}

void delayUs(unsigned long uS)
{
  unsigned long time_now = micros();
      while(micros() < time_now + uS);
}
