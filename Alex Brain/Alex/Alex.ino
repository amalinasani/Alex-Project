#include <stdarg.h>
#include <math.h>
#include <avr/sleep.h>
#include <serialize.h>

#include "packet.h"
#include "constants.h"
#include "circular.h"

// Power reduction
#define PRR_TWI_MASK 0b10000000
#define PRR_SPI_MASK 0b00000100
#define ADCSRA_ADC_MASK 0b10000000
#define PRR_ADC_MASK 0b00000001
#define PRR_TIMER2_MASK 0b01000000
#define PRR_TIMER0_MASK 0b00100000
#define PRR_TIMER1_MASK 0b00001000
#define SMCR_SLEEP_ENABLE_MASK 0b00000001
#define SMCR_IDLE_MODE_MASK 0b11110001

// Alex's length and breadth in cm
#define ALEX_LENGTH 6.5
#define ALEX_BREADTH 13 

// Alex's diagonal
float alexDiagonal = 0.0;

// Alex's turning circumference
float alexCirc = 0.0;

//timer for alex's speed adjustment
unsigned long int current_time = 0;
unsigned long int prev_time = 0;

typedef enum
{
  STOP = 0,
  FORWARD = 1,
  BACKWARD = 2,
  LEFT = 3,
  RIGHT = 4,

}   TDirection;


volatile TDirection dir = STOP;

/*
 * Alex's configuration constants
 */

// Number of ticks per revolution from the 
// wheel encoder.

#define COUNTS_PER_REV      196

// Wheel circumference in cm.
// We will use this to calculate forward/backward distance traveled 
// by taking revs * WHEEL_CIRC

#define WHEEL_CIRC          20.4

// Motor control pins. You need to adjust these till
// Alex moves in the correct direction
#define LF                  6   // Left forward pin
#define LR                  5   // Left reverse pin
#define RF                  10  // Right forward pin
#define RR                  11  // Right reverse pin

/*
 *    Alex's State Variables
 */

// Store the ticks from Alex's left and
// right encoders.
volatile unsigned long leftForwardTicks;
volatile unsigned long rightForwardTicks;
volatile unsigned long leftReverseTicks;
volatile unsigned long rightReverseTicks;

//Left and right encoder ticks for turning
volatile unsigned long leftForwardTicksTurns;
volatile unsigned long leftReverseTicksTurns;
volatile unsigned long rightForwardTicksTurns;
volatile unsigned long rightReverseTicksTurns;


// Store the revolutions on Alex's left
// and right wheels
volatile unsigned long leftRevs;
volatile unsigned long rightRevs;

// Forward and backward distance traveled
volatile float forwardDist;
volatile float reverseDist;
volatile float leftangle;
volatile float rightangle;

//variables to keep track of whether we have moved a commanded distance
unsigned long deltaDist;
unsigned long newDist;

// Variables to keep track of turning angle
unsigned long deltaTicks;
unsigned long targetTicks;

/*
* Power reduction 
*/
 
void WDT_off(void)
{
  /* Global interrupt should be turned OFF here if not
  already done so */
  /* Clear WDRF in MCUSR */
  MCUSR &= ~(1<<WDRF);
  /* Write logical one to WDCE and WDE */
  /* Keep old prescaler setting to prevent unintentional
  time-out */
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  /* Turn off WDT */
  WDTCSR = 0x00;
  /* Global interrupt should be turned ON here if
  subsequent operations after calling this function do
  not require turning off global interrupt */
}

void setupPowerSaving()
{
  // Turn off the Watchdog Timer
  WDT_off();
  // Modify PRR to shut down TWI
  PRR |= PRR_TWI_MASK;
  // Modify PRR to shut down SPI
  PRR |= PRR_SPI_MASK;
  // Modify ADCSRA to disable ADC,
  ADCSRA &= ~ADCSRA_ADC_MASK;
  // then modify PRR to shut down ADC
  PRR |= PRR_ADC_MASK;
  // Set the SMCR to choose the IDLE sleep mode
  SMCR &= SMCR_IDLE_MODE_MASK;
  // Do not set the Sleep Enable (SE) bit yet
  // Set Port B Pin 5 as output pin, then write a logic LOW
  // to it so that the LED tied to Arduino's Pin 13 is OFF.
  DDRB |= B00100000;
  PORTB &= B11011111;
}

void putArduinoToIdle()
{
  // Modify PRR to shut down TIMER 0, 1, and 2
  PRR |= (PRR_TIMER0_MASK | PRR_TIMER1_MASK | PRR_TIMER2_MASK);
  // Modify SE bit in SMCR to enable (i.e., allow) sleep
  SMCR |= SMCR_SLEEP_ENABLE_MASK;
  // The following function puts ATmega328P’s MCU into sleep;
  // it wakes up from sleep when USART serial data arrives
  sleep_cpu();
  // Modify SE bit in SMCR to disable (i.e., disallow) sleep
  SMCR &= ~SMCR_SLEEP_ENABLE_MASK;
  // Modify PRR to power up TIMER 0, 1, and 2
  PRR &= ~(PRR_TIMER0_MASK | PRR_TIMER1_MASK | PRR_TIMER2_MASK);
}

/*
 * 
 * Alex Communication Routines.
 * 
 */

static circular buffer_tx;
static circular buffer_rx;

void speed_set(int left, int right, TDirection movement)
{

  GTCCR = (1<<TSM) | (1<<PSRASY) | (1<<PSRSYNC);

  switch(movement)
  {
    case FORWARD:
      OCR0A = left;
      OCR0B = 0;
      OCR1BH = 0;
      OCR1BL = right;
      OCR2A = 0;
      break;
    case BACKWARD:
      OCR0A = 0;
      OCR0B = left;
      OCR1BH = 0;
      OCR1BL = 0;
      OCR2A = right;
      break;

    case RIGHT:
      OCR0A = left;
      OCR0B = 0;
      OCR1BH = 0;
      OCR1BL = 0;
      OCR2A = right;
      break;

    case LEFT:
      OCR0A = 0;
      OCR0B = left;
      OCR1BH = 0;
      OCR1BL = right;
      OCR2A = 0;
      break;

    case STOP:
      OCR0A = 0;
      OCR0B = 0;
      OCR1BH = 0;
      OCR1BL = 0;
      OCR2A = 0;
      break;
    
  }

  GTCCR = 0;

  
}

int prev_count_left;
int prev_count_right;

void speed_adjust(TDirection movement)
{
  //function used to adjust the speed using P control
  float left_over_right, right_over_left;
  float p = 0.005;
  float result = 0.0;
  int curr_left;
  int curr_right;
  int curr_count_left, curr_count_right;
  int new_left, new_right;


  
  

  switch(movement) //depending on movement, read the different speed values
  {
    case FORWARD:
      curr_left = OCR0A;
      curr_right = OCR1BL;
      curr_count_left = leftForwardTicks;
      curr_count_right = rightForwardTicks;
      left_over_right = ((float)(curr_count_left - prev_count_left))/((float)(curr_count_right - prev_count_right));
      right_over_left = ((float)(curr_count_right - prev_count_right))/((float)(curr_count_left - prev_count_left));
      prev_count_left = curr_count_left;
      prev_count_right = curr_count_right;
      break;

    case BACKWARD:
      curr_left = OCR0B;
      curr_right = OCR2A;
      curr_count_left = leftReverseTicks;
      curr_count_right = rightReverseTicks;
      left_over_right = ((float)(curr_count_left - prev_count_left))/((float)(curr_count_right - prev_count_right));
      right_over_left = ((float)(curr_count_right - prev_count_right))/((float)(curr_count_left - prev_count_left));
      prev_count_left = curr_count_left;
      prev_count_right = curr_count_right;
      break;

    case RIGHT:
      curr_left = OCR0A;
      curr_right = OCR2A;
      break;

    case LEFT:
      curr_left = OCR0B;
      curr_right = OCR1BL;
      break;
    
  }

  
  //we will try to adjust the right wheel first
  //the only time when we cannot adjust the right wheel is if its at max power and ratio is greater than one
  //or if it is at min power and ratio is less than one

  if (curr_right >= 255 && left_over_right > 1.0) //cannot speed up right wheel, slow down left instead
  {
    new_left = (int)((float)curr_left * right_over_left);
    new_right = curr_right;
  }

  else if (curr_right <=75 && left_over_right < 1.0) //cannot slow down any futher, speed up left instead
  {
    new_left = (int)((float)curr_left * right_over_left);
    new_right = curr_right;
  }
  else
  {
    new_left = curr_left;
    new_right = (int)((float)curr_right * left_over_right);
  }
  

  


  speed_set(new_left, new_right, movement);

  
}
 
TResult readPacket(TPacket *packet)
{
    // Reads in data from the serial port and
    // deserializes it.Returns deserialized
    // data in "packet".
    
    char buffer[PACKET_SIZE];//store packet info retrieved from serial port
    int len;

    len = readSerial(buffer);

    if(len == 0)
      return PACKET_INCOMPLETE;
    else
      return deserialize(buffer, len, packet);
    
}

void sendStatus()
{
  // Implement code to send back a packet containing key
  // information like leftTicks, rightTicks, leftRevs, rightRevs
  // forwardDist and reverseDist
  // Use the params array to store this information, and set the
  // packetType and command files accordingly, then use sendResponse
  // to send out the packet. See sendMessage on how to use sendResponse.
  //
  TPacket statusPacket;
  statusPacket.packetType = PACKET_TYPE_RESPONSE;
  statusPacket.command = RESP_STATUS;
  statusPacket.params[0] = leftForwardTicks;
  statusPacket.params[1] = rightForwardTicks;
  statusPacket.params[2] = leftReverseTicks;
  statusPacket.params[3] = rightReverseTicks;
  statusPacket.params[4] = leftForwardTicksTurns;
  statusPacket.params[5] = rightForwardTicksTurns;
  statusPacket.params[6] = leftReverseTicksTurns;
  statusPacket.params[7] = rightReverseTicksTurns;
  statusPacket.params[8] = forwardDist;
  statusPacket.params[9] = reverseDist;
  statusPacket.params[10] = leftangle;
  statusPacket.params[11] = rightangle;
  sendResponse(&statusPacket);
}

void sendMessage(const char *message)
{
  // Sends text messages back to the Pi. Useful
  // for debugging.
  
  TPacket messagePacket;
  messagePacket.packetType=PACKET_TYPE_MESSAGE;
  strncpy(messagePacket.data, message, MAX_STR_LEN);
  sendResponse(&messagePacket);
}

void dbprint(char *format, ...) {
  va_list args;
  char buffer[128];

  va_start(args, format);
  vsprintf(buffer, format, args);
  sendMessage(buffer);
  
}

void sendBadPacket()
{
  // Tell the Pi that it sent us a packet with a bad
  // magic number.

  TPacket badPacket;
  badPacket.packetType = PACKET_TYPE_ERROR;
  badPacket.command = RESP_BAD_PACKET;
  sendResponse(&badPacket);
  
}

void sendBadChecksum()
{
  // Tell the Pi that it sent us a packet with a bad
  // checksum.
  
  TPacket badChecksum;
  badChecksum.packetType = PACKET_TYPE_ERROR;
  badChecksum.command = RESP_BAD_CHECKSUM;
  sendResponse(&badChecksum);  
}

void sendBadCommand()
{
  // Tell the Pi that we don't understand its
  // command sent to us.
  
  TPacket badCommand;
  badCommand.packetType=PACKET_TYPE_ERROR;
  badCommand.command=RESP_BAD_COMMAND;
  sendResponse(&badCommand);

}

void sendBadResponse()
{
  TPacket badResponse;
  badResponse.packetType = PACKET_TYPE_ERROR;
  badResponse.command = RESP_BAD_RESPONSE;
  sendResponse(&badResponse);
}

void sendOK()
{
  TPacket okPacket;
  okPacket.packetType = PACKET_TYPE_RESPONSE;
  okPacket.command = RESP_OK;
  sendResponse(&okPacket);  
}

void sendResponse(TPacket *packet)
{
  // Takes a packet, serializes it then sends it out
  // over the serial port.
  char buffer[PACKET_SIZE];
  int len;

  len = serialize(buffer, packet, sizeof(TPacket));
  writeSerial(buffer, len);
}


/*
 * Setup and start codes for external interrupts and 
 * pullup resistors.
 * 
 */
// Enable pull up resistors on pins 2 and 3
void enablePullups()
{
  // Use bare-metal to enable the pull-up resistors on pins
  // 2 and 3. These are pins PD2 and PD3 respectively.
  // We set bits 2 and 3 in DDRD to 0 to make them inputs. 
  DDRD &= 0b11110011;
  PORTD |= 0b00001100;
}

// Functions to be called by INT0 and INT1 ISRs.
void leftISR()
{
    switch(dir)
  {
    case FORWARD:
      leftForwardTicks++;
      break;

    case BACKWARD:
      leftReverseTicks++;
      break;

    case LEFT:
      leftReverseTicksTurns++;
      break;

    case RIGHT:
      leftForwardTicksTurns++;
      break;

    default:
      break;
    
  }

  if(dir == FORWARD)
    forwardDist = (unsigned long) ((float) leftForwardTicks / COUNTS_PER_REV * WHEEL_CIRC);
  else if (dir == BACKWARD)
    reverseDist = (unsigned long) ((float) leftReverseTicks / COUNTS_PER_REV * WHEEL_CIRC);
  else if (dir == LEFT)
    leftangle = (unsigned long) (((float)leftReverseTicksTurns * 360.0 * WHEEL_CIRC)/( alexCirc * (float)COUNTS_PER_REV));
  else if (dir == RIGHT)
    rightangle = (unsigned long) (((float)leftForwardTicksTurns * 360.0 * WHEEL_CIRC)/( alexCirc * (float)COUNTS_PER_REV));
  
/*  if (!(leftTicks % COUNTS_PER_REV))
    leftRevs++;
  forwardDist = leftTicks * (float)WHEEL_CIRC/COUNTS_PER_REV;
  Serial.print("LEFT: ");
  Serial.print(leftRevs);
  Serial.print(" ");
  Serial.print(forwardDist);
  Serial.print(" ");
  Serial.println(leftTicks);*/
}

void rightISR()
{
  //I will assume that the vehicle wheels turn in the same direction when moving forward or backwards
  switch(dir)
  {
    case FORWARD:
      rightForwardTicks++;
      break;

    case BACKWARD:
      rightReverseTicks++;
      break;

    case LEFT:
      leftReverseTicksTurns++;
      break;

    case RIGHT:
      leftForwardTicksTurns++;
      break;

    default:
      break;

  }

  //dbprint("RIGHT: ");
  //dbprint("%d \n",rightForwardTicks);
}

// Set up the external interrupt pins INT0 and INT1
// for falling edge triggered. Use bare-metal.
void setupEINT()
{
  // Use bare-metal to configure pins 2 and 3 to be
  // falling edge triggered. Remember to enable
  // the INT0 and INT1 interrupts.
  EIMSK |= 0b11;
  EICRA |= 0b1010;
}

// Implement the external interrupt ISRs below.
// INT0 ISR should call leftISR while INT1 ISR
// should call rightISR.
ISR (INT0_vect)
{
  leftISR();
}
ISR (INT1_vect)
{
  rightISR();
}




// Implement INT0 and INT1 ISRs above.

/*For this section, we will need to set it in such a way that only one of these will trigger at a time
 * in other words, if compA has a ocroa value of 128, comp b should have zero to produce constant low
 * D is left, B is right
 */

ISR(TIMER0_COMPA_vect) //turn on the left motor forward pin6
{
  //do nothing
    
}

ISR(TIMER0_COMPB_vect) //turn on the right motors
{
  //do nothing
    
}

ISR(TIMER1_COMPB_vect)
{
  //do nothing, already done from initial setup
}

ISR(TIMER2_COMPA_vect)
{
  //do nothing
}
/*
 * Setup and start codes for serial communications
 * 
 */
// Set up the serial connection. For now we are using 
// Arduino Wiring, you will replace this later
// with bare-metal code.


void setupSerial()
{
  // To replace later with bare-metal.
//  Serial.begin(9600);

  
  UBRR0L = 103;
  UBRR0H = 0;
  UCSR0C = 0b00000110;
  UCSR0A = 0;
}

// Start the serial connection. For now we are using
// Arduino wiring and this function is empty. We will
// replace this later with bare-metal code.

void startSerial()
{
  // Empty for now. To be replaced with bare-metal code
  // later on.
  UCSR0B = 0b10111000;
}


ISR(USART_RX_vect) //triggers whenever UDR0 is suddenly filled with something
{
  unsigned char data = UDR0;
  int result;

  //now that you have the data, transfer it to the rx buffer

  result = buffer_rx.write_buffer((void*) &data, sizeof(char));

  if (result == 2)
  {
    //code will fail over here as there is too much data in the buffer
  }
  
  
}

// Read the serial port. Returns the read character in
// ch if available. Also returns TRUE if ch is valid. 
// This will be replaced later with bare-metal code.


int readSerial(char *buffer) //the bufer here is where the data will be stored after transfer from buffer_rx to buffer
{

  int count=0;

  int result;

  do
  {
    result = buffer_rx.read_buffer((void*) &buffer[count], sizeof(char));

      if(result == 0) //information read successfully
        count++;
  } while (result == 0); //read until the result come back that there is no more info stored

      
  return count; //count is the number of bytes read
}

// Write to the serial port. Replaced later with
// bare-metal code

ISR(USART_UDRE_vect)
{
  unsigned char data;

  int result;

  result = buffer_tx.read_buffer((void*)&data, sizeof(char));

  if(result == 0)
    UDR0 = data;
  else
    //if(result == 1)
      UCSR0B &=0b11011111;
}

void writeSerial(const char *buffer, int len) //buffer is where the data source is at, len is how many bytes to transmit
{
  int result = 0;

  int i;

  for(i = 1; i < len && result == 0; i++) //written one byte at a time
  {
    result = buffer_tx.write_buffer((void*)&buffer[i], sizeof(char));
  }

  UDR0 = buffer[0];

  UCSR0B |= 0b00100000;
}

/*
 * Alex's motor drivers.
 * 
 */

// Set up Alex's motors. Right now this is empty, but
// later you will replace it with code to set up the PWMs
// to drive the motors.
void setupMotors()
{
  /* Our motor set up is:  
   *    A1IN - Pin 5, PD5, OC0B
   *    A2IN - Pin 6, PD6, OC0A
   *    B1IN - Pin 10, PB2, OC1B
   *    B2In - pIN 11, PB3, OC2A
   */

  
  
  //setting up of timers
  TCNT0 = 0; //initial counter
  TCNT1H = 0;
  TCNT1L = 0;
  TCNT2 = 0;

  
  TCCR0A = 0b10100001; //we shall set the PWM pins at pd6 and pd7 to be under normal operation for now, Phase correct PWM
  TCCR0B = 0b00000011; // set clock prescalar to 64

  TCCR1A = 0b00100001;//PWM operation 8 bit only
  TCCR1B = 0b00000011;//set clock prescalar to 64

  TCCR2A = 0b10000001;//pwm operation
  TCCR2B = 0b00000100;//set clock prescalar to 64

  
  TIMSK0 |= 0b00000111; //OCOA, OCOB, overflow for millis
  TIMSK1 |= 0b00000100; //OC1B only
  TIMSK2 |= 0b00000010; //OC2A only
  
  


}

// Start the PWM for Alex's motors.
// We will implement this later. For now it is
// blank.
void startMotors()
{
  OCR0A = 0; //set the duty cycle left forward
  OCR0B = 0;
  OCR1BH = 0;
  OCR1BL = 0;
  OCR2A = 0;

  //pin setup, set these pins to output for pin 5, 6, 10, 11
  DDRD |= 0b01100000;
  DDRB |= 0b00001100;

  //port setup. Start all with a value of zero first
  PORTB &= 0b11110011;
  PORTD &= 0b10011111;
}

// Convert percentages to PWM values
int pwmVal(float speed)
{
  if(speed < 0.0)
    speed = 0;

  if(speed > 100.0)
    speed = 100.0;

  return (int) ((speed / 100.0) * 255.0);
}

// Move Alex forward "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// move forward at half speed.
// Specifying a distance of 0 means Alex will
// continue moving forward indefinitely.
void forward(float dist, float speed)
{

  if (dist > 0)
    deltaDist = dist;
  else
    deltaDist = 9999999;

  newDist = forwardDist + deltaDist;
  dir = FORWARD;
  int val = pwmVal(speed);

  // For now we will ignore dist and move
  // forward indefinitely. We will fix this
  // in Week 9.

  // LF = Left forward pin, LR = Left reverse pin
  // RF = Right forward pin, RR = Right reverse pin
  // This will be replaced later with bare-metal code.

  speed_set(val, val, FORWARD);
  prev_time = millis();
  prev_count_left = 0;
  prev_count_right = 0;
  clearCounters();
  
  /*
  analogWrite(LF, val);
  analogWrite(RF, val);
  analogWrite(LR,0);
  analogWrite(RR, 0);
*/
  
}

// Reverse Alex "dist" cm at speed "speed".f

// "speed" is expressed as a percentage. E.g. 50 is
// reverse at half speed.
// Specifying a distance of 0 means Alex will
// continue reversing indefinitely.
void reverse(float dist, float speed)
{
  if (dist > 0)
    deltaDist = dist;
  else
    deltaDist = 9999999;

  newDist = reverseDist + deltaDist;
  dir = BACKWARD;
  int val = pwmVal(speed);

  // For now we will ignore dist and 
  // reverse indefinitely. We will fix this
  // in Week 9.

  // LF = Left forward pin, LR = Left reverse pin
  // RF = Right forward pin, RR = Right reverse pin
  // This will be replaced later with bare-metal code.

  speed_set(val, val, BACKWARD);
  prev_time = millis();
  prev_count_left = 0;
  prev_count_right = 0;
  clearCounters();
  /*
  analogWrite(LR, val);
  analogWrite(RR, val);
  analogWrite(LF, 0);
  analogWrite(RF, 0);
  */
}

unsigned long computeDeltaTicks(float ang) 
{
  unsigned long ticks = (unsigned long) ((ang * alexCirc * COUNTS_PER_REV) / (360.0 * WHEEL_CIRC));

  return ticks;
}

// Turn Alex left "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn left indefinitely.
void left(float ang, float speed)
{
    
  dir = LEFT;
  int val = pwmVal(speed);

  if (ang == 0)
    deltaTicks = 99999999;
  else 
    deltaTicks = computeDeltaTicks(ang);

  targetTicks = leftReverseTicksTurns + deltaTicks;

  prev_count_left = 0;
  prev_count_right = 0;
  clearCounters();
  
  speed_set(val, val, LEFT);
/*
  analogWrite(LR, val);
  analogWrite(RF, val);
  analogWrite(LF, 0);
  analogWrite(RR, 0);*/
}

// Turn Alex right "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn right indefinitely.
void right(float ang, float speed)
{
  dir = RIGHT;
  int val = pwmVal(speed);

  if (ang == 0) 
    deltaTicks = 99999999;

  else 
    deltaTicks = computeDeltaTicks(ang);

  targetTicks = leftForwardTicksTurns + deltaTicks;

  // For now we will ignore ang. We will fix this in Week 9.
  // We will also replace this code with bare-metal later.
  // To turn right we reverse the right wheel and move
  // the left wheel forward.

  prev_count_left = 0;
  prev_count_right = 0;
  clearCounters();
 
  speed_set(val, val, RIGHT);
  /*
  analogWrite(RR, val);
  analogWrite(LF, val);
  analogWrite(LR, 0);
  analogWrite(RF, 0);
  */
}

// Stop Alex. To replace with bare-metal code later.
void stop()
{
  dir = STOP;

  
  speed_set(0, 0, STOP);
  //OCR0B = 0;
  //OCR2A = 0;
  //OCR2B = 0;
  /*
  analogWrite(LF, 0);
  analogWrite(LR, 0);
  analogWrite(RF, 0);
  analogWrite(RR, 0);
  */
}

/*
 * Alex's setup and run codes
 * 
 */

/* 
// Store the ticks from Alex's left and
// right encoders.
volatile unsigned long leftForwardTicks;
volatile unsigned long rightForwardTicks;
volatile unsigned long leftReverseTicks;
volatile unsigned long rightReverseTicks;

//Left and right encoder ticks for turning
volatile unsigned long leftForwardTicksTurns;
volatile unsigned long leftReverseTicksTurns;
volatile unsigned long rightForwardTicksTurns;
volatile unsigned long rightReverseTicksTurns;


// Store the revolutions on Alex's left
// and right wheels
volatile unsigned long leftRevs;
volatile unsigned long rightRevs;

// Forward and backward distance traveled
volatile float forwardDist;
volatile float reverseDist; */

// Clears all our counters
void clearCounters()
{
  leftForwardTicks = 0;
  rightForwardTicks = 0;
  leftReverseTicks = 0;
  rightReverseTicks = 0;
  leftForwardTicksTurns = 0;
  leftReverseTicksTurns = 0;
  rightForwardTicksTurns = 0;
  rightReverseTicksTurns = 0;
  leftRevs=0;
  rightRevs=0;
  forwardDist=0;
  reverseDist=0; 
  leftangle = 0;
  rightangle = 0;
}

// Clears one particular counter
void clearOneCounter(int which)
{
  clearCounters();
}
// Intialize Vincet's internal states

void initializeState()
{
  clearCounters();
}

void handleCommand(TPacket *command)
{

  switch(command->command)
  {
    
    // For movement commands, param[0] = distance, param[1] = speed.
    case COMMAND_FORWARD:
        sendOK();
        forward((float) command->params[0], (float) command->params[1]);
      break;

    case COMMAND_REVERSE:
        sendOK();
        reverse((float) command->params[0], (float) command->params[1]);
      break;

    case COMMAND_TURN_LEFT:
        sendOK();
        left((float) command->params[0], (float) command->params[1]);
      break;

    case COMMAND_TURN_RIGHT:
        sendOK();
        right((float) command->params[0], (float) command->params[1]);
      break;

    case COMMAND_STOP:
        sendOK();
        stop();
      break;

    case COMMAND_GET_STATS:
        sendStatus();
      break;

    case COMMAND_CLEAR_STATS:
        clearCounters();
      break;
        
    default:
      sendBadCommand();
  }
}

void waitForHello()
{
  int exit=0;

  while(!exit)
  {
    TPacket hello;
    TResult result;
    
    do
    {
      result = readPacket(&hello);
    } while (result == PACKET_INCOMPLETE);

    if(result == PACKET_OK)
    {
      if(hello.packetType == PACKET_TYPE_HELLO)
      {
     

        sendOK();
        exit=1;
      }
      else
        sendBadResponse();
    }
    else
      if(result == PACKET_BAD)
      {
        sendBadPacket();
      }
      else
        if(result == PACKET_CHECKSUM_BAD)
          sendBadChecksum();
  } // !exit
}

void setup() {
  // put your setup code here, to run once:
  alexDiagonal = sqrt((ALEX_LENGTH * ALEX_LENGTH) + (ALEX_BREADTH * ALEX_BREADTH));
  alexCirc = PI * alexDiagonal;

  cli();
  setupEINT();
  setupSerial();
  startSerial();
  setupMotors();
  startMotors();
  enablePullups();
  initializeState();


  sei();
}

void handlePacket(TPacket *packet)
{
  switch(packet->packetType)
  {
    case PACKET_TYPE_COMMAND:
      handleCommand(packet);
      break;

    case PACKET_TYPE_RESPONSE:
      break;

    case PACKET_TYPE_ERROR:
      break;

    case PACKET_TYPE_MESSAGE:
      break;

    case PACKET_TYPE_HELLO:
      break;
  }
}



void loop() {

// Uncomment the code below for Step 2 of Activity 3 in Week 8 Studio 2

  //forward(1, 100);

// Uncomment the code below for Week 9 Studio 2


  //dbprint("Hello world");
 // put your main code here, to run repeatedly:
  TPacket recvPacket; // This holds commands from the Pi
  
  TResult result = readPacket(&recvPacket);
  
  if(result == PACKET_OK)
  {
    //forward(2.0, 100);
    handlePacket(&recvPacket);
  }
  else
    if(result == PACKET_BAD)
    {
      //reverse(2.0, 100);
      sendBadPacket();
    }
    else
      if(result == PACKET_CHECKSUM_BAD)
      {
        //reverse(2.0, 100);
        sendBadChecksum();
      } 

  if(deltaDist > 0)
  {
    if (dir == FORWARD)
    {
      if(forwardDist >= newDist)
      {
        deltaDist = 0;
        newDist = 0;
        stop();
      }

      
      current_time = millis();
      if(current_time - prev_time > 250)
      {
        speed_adjust(FORWARD);
        prev_time = millis();
      }
    }
    else if (dir == BACKWARD)
    {
      if(reverseDist >= newDist)
      {
        deltaDist = 0;
        newDist = 0;
        stop();
      }
      current_time = millis();
      if(current_time - prev_time > 1000)
      {
        speed_adjust(BACKWARD);
        prev_time = millis();
      }
    }

    else if(dir == STOP)
    {
       deltaDist = 0;
       newDist = 0;
       stop();
       putArduinoToIdle();
    }
  }

  if (deltaTicks > 0) 
  {
    if (dir == LEFT) 
    {
      if (leftReverseTicksTurns >= (targetTicks))
      {
        deltaTicks = 0;
        targetTicks = 0;
        stop();
      }
    }
    else if (dir == RIGHT) 
    {
      if (leftForwardTicksTurns >=  targetTicks)
      {
        deltaTicks = 0;
        targetTicks = 0;
        stop();
      }
    }
    else if (dir == STOP)
    {
      deltaTicks = 0;
      targetTicks = 0;
      stop();
      putArduinoToIdle();
    }
  }    
}
