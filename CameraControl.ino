/*
* Inspired by Arduino Pan/Tilt http://vimeo.com/40569055
* This program controls the Bescor MP-101 Pan/Tilt system using the Wii Nunchuck
* and LAN-C camera zoom/power
*
* Uses Nunchuk Library created by Gabriel Bianconi, http://www.gabrielbianconi.com/
*
* Developed by Ray Cole
* Jan 22, 2020
* Leander Church of Christ
* LANC Interrupt Service Routine is original code to allow smooth pan/tilt during zoom in/out
*/
 
#include "ArduinoNunchuk.h"                               // The library used by the Nunchuck
#include <util/atomic.h>                                  // needs atomic

// ----------------------------------------------------------------------------------------------
// Define the I/O pins
// Bescor MP-101
// ----------------------------------------------------------------------------------------------
#define leftPin  10                                       // pan left
#define rightPin  9                                       // pan right
#define upPin    11                                       // tilt up
#define downPin   3                                       // tilt down

// LANC signal
#define lancPin   2                                       // lanc (camera control) - needs to be on pin 2 for interrupts

// ----------------------------------------------------------------------------------------------
// Define nunchuk controller ranges and such
// ----------------------------------------------------------------------------------------------
#define MIN_CENTER    15                                  // minimum movement to come off center
#define CENTER_POINT 128                                  // proper center reading point
#define X_CENTER     (CENTER_POINT - 123)                 // based on nunchuk showing (123,136) at startup
#define Y_CENTER     (CENTER_POINT - 136)                 // based on nunchuk showing (123,136) at startup
#define Z_CENTER     (CENTER_POINT - 0)                   // z-axis (not calibrarted)
#define RANGE        100                                  // full range
#define RANGE_UP      89                                  // up range is limited on my nunchuk
#define RANGE_DOWN   100                                  // down range is good
#define RANGE_LEFT   100                                  // left range is good
#define RANGE_RIGHT   99                                  // right is almost there

#define MODE_TIME        500                              // mode change time
#define POWEROFF_TIME   5000                              // power off time (how long to hold both buttons to power off)
#define INACTIVE_TIMEOUT 120                              // inactive timeout in minutes

#define ZOOM_STEP       20                                // zoom speed stepping (how far controller has to be moved for each change in zoom speed)
#define ZOOM_SPEEDS      8                                // zoom speeds (there are 8 slow and 8 fast 

ArduinoNunchuk nunchuk;                                   // The instance of the Nunchuck class
// ----------------------------------------------------------------------------------------------

// ----------------------------------------------------------------------------------------------
// LANC control
// Only tested with a single Canon Vixia HF G21 NTSC camera
// Telegram timings may need adjustment for non-NTSC version of the camera
// Other camera modesl may also need telegram adjustment and additional statuses to be added.
// ----------------------------------------------------------------------------------------------
#define LANC_BIT_TICKS                                 26 // need this to be 104us between bits (16MHz / 64 * 26 = 104us)
                                                          // assumes scaling factor of 64 - default for timer 0 on Arduino Uno

#define LANC_MID_TICKS            (LANC_BIT_TICKS / 3)    // # ticks to place into a good spot for reading bits. 33% gives some timing wiggle room

#define TELEGRAM_SIG_MIN_USEC                    6000     // minimum usecs for telegram signal length
#define TELEGRAM_SIG_MAX_USEC                   20000     // maximum usecs for telegram signal length
 
#define LANC_STATE_RESET                            0     // reset (searching for initial telegram)
#define LANC_STATE_TELEGRAM                         1     // searching for telegram
#define LANC_STATE_STOPBIT                          2     // waiting for a stopbit
#define LANC_STATE_BYTE                             3     // processing byte data
#define LANC_STATE_END_BYTE                         4     // at end of byte

#define LANC_STATUS_STOP                          0x02    // stopped
#define LANC_STATUS_RECORDING                     0x04    // recording
#define LANC_STATUS_PAUSED                        0x14    // paused

static void lancFalling();                                // LANC falling interrupt

class CameraLANC                                          // LANC camera implementation
{

private:

  volatile char itsPowered;                               // is it powered on?
  volatile byte itsStatus;                                // overall status

  volatile int  itsState;                                 // protocol state
  volatile int  itsBitCnt;                                // bit number being processed
  volatile int  itsSendCnt;                               // send counter
  volatile int  itsCurVal;                                // current value being sent
  
  volatile byte itsValue[2];                              // values to write
  volatile byte itsRead[8];                               // bytes for LANC

  volatile unsigned long itsWaitSt;                       // wait for low start time
    
  volatile bool itsSendRdy;                               // is send ready?
  volatile bool itsDone;                                  // sending of data has completed
  
  int           itsPin;                                   // I/O pin. Some sketches use 2 pins, this one uses one

public :
  
  CameraLANC()                                            // standard constructor
  {
    itsPowered = false;                                   // assume camera is not yet on
    itsPin = lancPin;                                     // set I/O pin
    pinMode(itsPin, INPUT_PULLUP);                        // set input pin for input
    itsStatus = 0;                                        // no status
    itsDone = true;                                       // finished transferring
    itsBitCnt = 0;                                        // bit counter
    itsSendRdy = false;                                   // not ready to send yet
    itsSendCnt = 0;                                       // no send counter yet
    itsWaitSt = 0;                                        // no time yet
    itsState = LANC_STATE_RESET;                          // in reset state for now
  }

  void Setup()                                            // Setup

  {                                                       // begin setup
    itsWaitSt = micros();                                 // get waiting time
    TIMSK0 |= _BV(OCIE0A);                                // turn on interrupt handler      
    attachInterrupt(digitalPinToInterrupt(itsPin),        // start
                    lancFalling,                          // monitor the pin
                    FALLING);                             // for falls
  }                                                       // end setup
  
  void PowerOn()                                          // turn camera on
  {
      pinMode(itsPin, OUTPUT);                            // writes to the LANC line     
      digitalWrite(itsPin, LOW);                          // set LANC line to +5V (short)
      delay(200);                                         // wait for a moment for camera to see this
      pinMode(itsPin, INPUT_PULLUP);                      // back to input
  }
    
  void Record()                                           // start recording
  {
      if (itsStatus != 0x04)                              // if not already recording    
         SendCode(0x18, 0x33);                            // toggle recording to record
  }
  
  void Pause()                                            // pause recording
  {
      if (itsStatus == 0x04)                              // if recording
         SendCode(0x18, 0x33);                            // toggle recording to pause
  }
  
  void PowerOff() { SendCode(0x18, 0x5e); };              // send command to power off (sleep)
  void ToggleDisplay() { SendCode(0x18, 0xb4); };         // toggle display
  void Menu() { SendCode(0x18, 0x9A); };                  // go to top menu
  void Select() { SendCode(0x18, 0xA2); };                // select item on screen
  void Left() { SendCode(0x18, 0xC4); };                  // move left
  void Right() { SendCode(0x18, 0xC2); };                 // move right
  void Up() { SendCode(0x18, 0x84); };                    // move up
  void Down() { SendCode(0x18, 0x86); };                  // move down
  void ToggleFocusMode(){ SendCode(0x28, 0x41); };        // toggle autofocus off/on
  void FocusNear() { SendCode(0x28, 0x47); };             // focus near
  void FocusFar() { SendCode(0x28, 0x45); };              // focus far
  bool IsPowered() { return itsPowered; };                // is it powered up?
  byte Status() { return itsStatus;  };                   // return status

  void SendCode(int type,int code)                        // send a code pair
  {

    while (itsPowered &&                                  // we have power and
           (itsSendRdy || !itsDone) &&                    // if we have another command pending
           (itsValue[0] != type && itsValue[1] != code)) delay(2); // and it isn't the same one, then wait <-- Would be nice to have a queue, but this rarely happens

    if (!itsSendRdy && itsDone)                           // able to send it?
    {                                                     // begin mark to send
      itsValue[0] = type;                                 // set bytes
      itsValue[1] = code;                                 // to
      itsSendRdy = true;                                  // send
    };                                                    // end mark to send
    
  }  

  void Zoom(int theVelocity)                              // zoom - velocity range is -8 to 8
  {
    
    int aCmdCat = 0x28;                                   // subcommand - video camera specific 
    int aSubCmd = (abs(theVelocity)-1) * 2;               // get base command (0x28 0x00 is slow zoom

    if (theVelocity < 0)                                  // if zooming out
      aSubCmd += 0x10;                                    // put into zoom out range 
  
    SendCode(aCmdCat, aSubCmd);                           // send zoom command
    
  }

  byte GetStatus()                                        // obtain the status
  {                                                       // begin obtain status
     byte aStatus;                                        // status return
     ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
     {
        aStatus = itsStatus;                              // grab the status safely
     }
     return(aStatus);                                     // return the status
  };                                                      // end obtain status

  void Display()                                          // display status information
  {
      static byte aLastStat = 0;
      if (aLastStat == itsStatus) return;
      aLastStat = GetStatus();
      if (IsPowered()) Serial.print("Camera is on, "); else Serial.print("Camera is off, ");
      Serial.print("Status "); Serial.print(Status(), HEX); Serial.println();
  }
 
  inline void Falling()                                   // signal from high to low
  {
      int aCurCnt = TCNT0;                                // get timer0 counter 
      bool aSetTimer = false;                             // don't set timer by default

      unsigned long aCurTime = micros();                  // grab current time

      if (!itsPowered)                                    // not known to be powered on?
      {                                                   // begin mark powered on
          itsPowered = true;                              // we are powered on now
          itsWaitSt = aCurTime;                           // fresh start for waiting
          itsState = LANC_STATE_RESET;                    // we are in a reset state
          Serial.println("Camera powered up");
          return;                                         // wait for it to signal some more
      };                                                  // end mark powered on  

      unsigned long aWaited = aCurTime - itsWaitSt;       // compute time between falls
               
      switch(itsState)                                    // check state
      {                                                   // begin cases of state

        default                                         : // default
         break;                                           // end do nothing

        case LANC_STATE_TELEGRAM                        : // ready for new telegram
        case LANC_STATE_RESET                           : // ready for reset
          if (aWaited > TELEGRAM_SIG_MIN_USEC &&          // was the duration of the telegram signal
              aWaited < TELEGRAM_SIG_MAX_USEC)            // something that looks reasonable?
          {                                               // begin telegram found
             if (itsState == LANC_STATE_RESET)            // was it a reset
                itsSendCnt = 0;                           // reset send counter
             itsBitCnt = 0;                               // starting with first bit
             itsWaitSt = aCurTime;                        // set wait start time

             if (itsSendCnt > 3 && itsSendRdy)            // are values waiting to be written and are we ready?
             {                                            // begin pickup values to write
                itsDone = false;                          // we are not done
                itsSendRdy = false;                       // no more value pending
                itsSendCnt = 0;                           // we haven't sent it yet
             }                                            // end pickup values to write
             aSetTimer = true;                            // set the timer (telegram detection really detects the first stop bit after a telegram)
          }                                               // end telegram found
          else                                            // otherwise
          {                                               // begin reset
            if (itsState == LANC_STATE_TELEGRAM)          // were we expecting a telegram?
              Serial.println("Reset");                    // let someone knoe we went off the rails...
             itsState = LANC_STATE_RESET;                 // reset - did not find the telegram
             itsWaitSt = aCurTime;                        // set wait start time
          };                                              // end reset
          break;                                          // end telegram processing

        case LANC_STATE_STOPBIT                         : // stopbit
          aSetTimer = true;                               // we will set the timer
      };                                                  // end cases of state

      if (aSetTimer)                                      // need to set timer?
      {                                                   // begin set timer
          itsWaitSt = aCurTime;                           // reset wait start time
          
          if (itsBitCnt == 16 && itsSendCnt >= 2)         // wrote the command twice?
             itsDone = true;                              // we are done - ready for another command

          itsState = LANC_STATE_BYTE;                     // processing bytes now
      
          int aNewCmp = aCurCnt + LANC_BIT_TICKS;         // prepare new compare register value
      
          if (itsBitCnt >= 16 && itsBitCnt < 64)          // in read range?
            aNewCmp += LANC_MID_TICKS;                    // want to land in the middle of the bits we read
                     
          unsigned aSave = TCCR0A;                        // save timer setup
          TCCR0A = 0;                                     // turn timer off for just a brief moment
          OCR0A = aNewCmp;                                // set new register
          TCCR0A = aSave;                                 // enable timer  
      };                                                  // end set timer

  }
  
  inline int ProcessBits()                                // process bits
  {                                                       // begin interrupt routine

    int aRc = 0;                                          // assume exit without adjustment to compare register

    switch(itsState)                                      // test state
    {                                                     // begin cases of state
      case LANC_STATE_BYTE                              : // processing bytes
      
        aRc = LANC_BIT_TICKS;                             // assume we will want to process again in 1 bit's width of time

        if (itsBitCnt >= 16)                              // in area where we should read bytes?
        {                                                 // begin read bytes    
            int aByte = itsBitCnt / 8;                    // get byte number        
            int aBit = itsBitCnt % 8;                     // get remainder to identify bit
            if (aBit == 0)                                // if low        
                itsRead[aByte] = 0;                       // clear status byte
            if (digitalRead(itsPin) == LOW)               // == *LOW* because bits inverted in LANC         
                itsRead[aByte] |= 1 << aBit;              // record bit high
        }                                                 // end read bytes
        else                                              // otherwise
        {                                                 // begin write data
            if (itsBitCnt == 0 || itsBitCnt == 8)         // new byte boundary?
            {                                             // begin pickup byte to write
                pinMode(itsPin,OUTPUT);                   // set for output 
                itsCurVal = itsValue[itsBitCnt / 8];      // grab value to write
            };                                            // end pickup byte to write
            if (!itsDone)                                 // actual data to write?
            {                                             // begin write bit
                digitalWrite(itsPin, !(itsCurVal & 1));   // write reversed
                itsCurVal >>= 1;                          // advance to next bit
            }                                             // end write bit
            else                                          // otherwise
                digitalWrite(itsPin, !0);                 // write zeroes
        };                                                // end write data

        itsBitCnt ++;                                     // bump bit counter

        if (itsBitCnt == 64 && itsSendCnt < 4)            // at the end and send counter not too high?
           itsSendCnt ++;                                 // nope, bump counter     

        if (itsBitCnt >= 64)                              // finished with read bits?
        {                                                 // begin setup status
            itsStatus = itsRead[4];                       // set overall status
            if (itsStatus != LANC_STATUS_STOP &&          // if status  
                itsStatus != LANC_STATUS_RECORDING &&     // is not something we would
                itsStatus != LANC_STATUS_PAUSED)          // expect (add to this if your camera shows another valid status...or disabled this check as it is just a safety net...)
            {                                             // begin reset - status seems dubious
                itsWaitSt = micros();                     // set transition time
                itsState = LANC_STATE_RESET;              // reset - we may be off in the weeds        
                Serial.print("Bad status ");              // indicate status looks wrong
                Serial.println(itsStatus, HEX);           // write out the status we obtained
                aRc = 0;                                  // just need to return
            };                                            // end reset
        }                                                 // end setup status
                                                          
        if (aRc != 0 &&                                   // normal return so far
            (itsBitCnt % 8) == 0)                         // end of a byte?
        {                                                 // begin end of byte
            itsState = LANC_STATE_END_BYTE;               // wait on end of byte
            if (itsBitCnt > 16)                           // if in reading range
              aRc -= LANC_MID_TICKS;                      // reduce the ticks because we are reading halfway into the time for each bit      
        };                                                // end of byte        
        break;                                            // end processing bytes

      case LANC_STATE_END_BYTE                          : // end of byte
        pinMode(itsPin, INPUT_PULLUP);                    // toggle back to read mode
        if (itsBitCnt == 64)                              // last bit processed?
           itsState = LANC_STATE_TELEGRAM;                // new telegram should be next
        else                                              // otherwise
           itsState = LANC_STATE_STOPBIT;                 // wait for stopbit
        itsWaitSt = micros();                             // start of wait
        break;                                            // end byte

      default                                           : // default
        if (itsPowered && micros() - itsWaitSt > 1000000) // powered on but long time since a pulse has been observed?
        {                                                 // begin powered off
          itsStatus = 0;                                  // status not read
          itsPowered = false;                             // power has failed (or at least communication has)
         }                                                // end powered off
        break;                                            // end default
      
    };                                                    // end cases of state
    return (aRc);                                         // return result   
  }                                                       // end interrupt routine
  
} Camera;

// ---------------------------------------------------------------------------------------------- 
// LANC Falling signal interrupt routine
// ----------------------------------------------------------------------------------------------
void lancFalling()
{
  Camera.Falling();
}

// ---------------------------------------------------------------------------------------------- 
// LANC Timer Interrupt routine for reading/writing bits
// This calls will adjust the compare register to setup for the next bit to process
// ----------------------------------------------------------------------------------------------
ISR(TIMER0_COMPA_vect)                                    // setup for compare register A on timer 0
{                                                         // begin LANC interrupt

    unsigned anEntry = OCR0A;                             // grab last compare
    
    int aNext = Camera.ProcessBits();                     // call routine to process bits
    
    if (aNext != 0)                                       // need to adjust?
    {                                                     // begin set next compare 
        aNext = aNext + anEntry;                          // set next relative to compare register on entry
        unsigned aSave = TCCR0A;                          // save timer setup
        TCCR0A = 0;                                       // turn timer off for just a brief moment
        OCR0A = aNext;                                    // set new register
        TCCR0A = aSave;                                   // resume timer
    }                                                     // end set next compare
}                                                         // end LANC interrupt

// -------------------------------------------------------------------------------------------------------

// -------------------------------------------------------------------------------------------------------
// Button Class
// Contains information for buttons
//--------------------------------------------------------------------------------------------------------
class Button
{
  public :

    char          itsDown;                                  // down?
    char          itsFirstDown;                             // first time down was checked?

    unsigned long itsDownTime;                              // down start time
  
  Button()
  {
     itsDown = false;                                       // not down yet
     itsFirstDown = false;                                  // nobody has checked us yet
     itsDownTime = 0;                                       // not down yet
  }

  char Down() { return itsDown; }                           // is button down?

  char FirstDown()                                          // first detection of button down? (rather than we've seen it down before, it is being held)
  {       
    if (itsDown && !itsFirstDown)                           // if it is down and this is the first time the caller will have seen it
    {                                                       // begin indicate this is the first down
      itsFirstDown = true;                                  // someone has seen it down now, so don't report it as first down again
      return true;                                          // return this is the first time to report down
    };                                                      // end indicate this is the first down
    
    return false;                                           // return not the first time to be reported down
  }

  bool Update(char theState)                                // update

  {

     if (theState)                                          // button is down
     {
        if (!itsDown)
           itsDownTime = millis();                          // get down time           
        itsDown = true;                                     // mark button down
     }
     else                                                   // otherwise
     {                                                      // begin mark as down
        itsDown = false;                                    // not down
        itsFirstDown = false;                               // nobody has checked yet
     };                                                     // end mark as down

     return itsDown;                                        // action if button is hit
     
  }
      
};

// -------------------------------------------------------------------------------------------------------
// Specific button classes
// Sets input 
//--------------------------------------------------------------------------------------------------------
class zButtonClass : public Button {
   public :
   zButtonClass() : Button() { };
   bool Input() { return(Update(nunchuk.zButton)); }
} zButton;

class cButtonClass : public Button {
   public:
   cButtonClass() : Button() { };
   bool Input() { return(Update(nunchuk.cButton)); }
} cButton;


// -------------------------------------------------------------------------------------------------------
// Zoom Class
// Contains information for zooming
//--------------------------------------------------------------------------------------------------------
class ZoomCtrl
{
  public :

    int itsNeutral;                                         // neutral point for zooming
    int itsLastZoom;                                        // last zoom

  ZoomCtrl()
  {
     itsNeutral = 0;                                        // no neatural point yet
     itsLastZoom = 0;                                       // last zoom 
  }

  bool Input()                                              // process input
  {

     bool anAction = false;                                 // assume no action
    
     if ((zButton.Down() && !cButton.Down())           ||   // if zoom is down and other button isn't
         (cButton.Down() && !zButton.Down()))               // (Z for up/down, c for roll - trying the two out)

     {                                                      // begin process down

        anAction = true;                                    // we have a zoom action

        int aPos = nunchuk.accelY;                          // get axis value
        int aFirstDown = false;

        if (cButton.Down())                                 // if using c button

        {
           aPos = nunchuk.accelX;                           // use roll control
           aFirstDown = cButton.FirstDown();
        }
        else
        {
           aFirstDown = zButton.FirstDown();
        }

        if (aFirstDown)                                     // was it just now pressed?
        {                                                   // begin initiate zoom
           //Serial.println("Zoom button down.");
           itsNeutral = aPos;                               // set current Y as neutral
           itsLastZoom = 0;                                 // reset zoom
        };                                                  // end initiate zoom

        int aRead = aPos - itsNeutral;                      // find relative to neutral
        int aZoomVelocity = itsLastZoom;

        if (abs(aRead) >= ZOOM_STEP)                        // has it moved enough yet?
        
        {

          itsNeutral = aPos;                                // establish new neutral position
  
          aZoomVelocity = itsLastZoom +                     // bump zoom
                                (aRead / abs(aRead));       // speed one step

          aZoomVelocity = constrain(aZoomVelocity,          // constraint to proper range
                                    -ZOOM_SPEEDS,
                                    ZOOM_SPEEDS);

          if (itsLastZoom != aZoomVelocity)                 // is this a change?
          {
            //Serial.print("Zoom ");                          // write about it
            //Serial.println(aZoomVelocity);                  // negative = zoom out, positive is zoom in
          }
          
        }
        
        if (aZoomVelocity == 0 && itsLastZoom != 0)         // transition to no zoom?
        {        
           //Serial.println("Zoom is neutral");
        };

        if (aZoomVelocity != 0)                             // zooming?
        {                                                   // begin send command          
          Camera.Zoom(aZoomVelocity);                       // zoom          
        }
        itsLastZoom = aZoomVelocity;                        // save last zoom
      
     }                                                      // end process down

     else                                                   // button not down

     {                                                      // begin mark up


        if (itsNeutral != 0)                                // if set
        {
           //Serial.println("Zoom stopped.");
        };

        itsNeutral = 0;                                     // all stopped
        itsLastZoom = 0;                                    // last zoom - none

     }

     return(anAction);
     
  }
  
} Zoom;

// -------------------------------------------------------------------------------------------------------
// PWM Motor Class
// Must use automatic PWM because doing manual PWM with interrupts will cause LAN-C to fail as LAN-C is
// timing-dependent.
//--------------------------------------------------------------------------------------------------------
class PwmMotor
{
public:
 
  int itsVelocity;                                        // Velocity
  int itsDrive;                                           // Current pin to drive
  int itsPin[2];                                          // Pins assigned to motor
  int itsLow;                                             // Low range for PWM (percentage)
  int itsHigh;                                            // High range for PWM (percentage)
  
  PwmMotor(int thePin1,                                   // First pin to control
           int thePin2,                                   // Second pin to control
           int theLow,                                    // Low range for PWM (percentage)
           int theHigh)                                   // High range for PWM (percentage)
  {
     itsVelocity = 0;                                     // Do not move yet
     itsDrive = -1;                                       // No pin to drive 
     itsLow = theLow;                                     // Low range
     itsHigh = theHigh;                                   // High range
     itsPin[0] = thePin1;                                 // Set pin 1
     itsPin[1] = thePin2;                                 // Set pin 2
     digitalWrite(itsPin[0], LOW);                        // take low
     digitalWrite(itsPin[1], LOW);                        // take low
     
  }

  void Setup()
  {
     analogWrite(itsPin[0], 0);                           // indicate PWM pin
     analogWrite(itsPin[1], 0);                           // indicate PWM pin
     digitalWrite(itsPin[0], LOW);                        // set low (PWM off for now)
     digitalWrite(itsPin[1], LOW);                        // set low (PWM off for now)
  }

  void Stop()                                             // stop
  {
     Setup();                                             // Setup will make it stop
  }

  bool Move(int theVelocity)                              // Move based on velocity (-100 to 100, based upon input control)

  {

    //if (abs(theVelocity) < MIN_CENTER) theVelocity = 0;   // allow some play for centering of control

    theVelocity = constrain(theVelocity, -RANGE, RANGE);  // constraint within our design limts

    if (theVelocity == itsVelocity) return false;         // no change? Nothing to do here - move on

    // Attempt to smooth out speed variations
    //itsVelocity = (itsVelocity + theVelocity) / 2;
    if (theVelocity > itsVelocity) itsVelocity = min(itsVelocity + 2, RANGE);
    else if (theVelocity < itsVelocity) itsVelocity = max(itsVelocity - 2, -RANGE);
    else itsVelocity = 0;

    if (Zoom.itsLastZoom != 0)                            // if zooming
      itsVelocity = theVelocity;                          // don't worry about smooth

    int aDrive = -1;                                      // assume off
    
    if (itsVelocity > 0)                                  // negative velocity
      aDrive = itsPin[1];                                 // drive second pin
    else                                                  // otherwise
      if (itsVelocity < 0) aDrive = itsPin[0];            // drive first pin if negative

    if (itsDrive != aDrive && itsDrive != -1)             // change in pins
    {
         //Serial.print(itsDrive); Serial.println(" : Off");
         digitalWrite(itsDrive, LOW);                     // drop old pin low         
    }
      
    itsDrive = aDrive;                                    // set new drive pin
      
    if (itsDrive == -1) return false;                     // not being driven? get out of here!

    if (abs(itsVelocity) > MIN_CENTER)                    // allow some play for centering of control
       analogWrite(itsDrive, map(abs(itsVelocity), MIN_CENTER, 100, (float) 255 * itsLow / 100, (float) 255 * itsHigh / 100));
    else
    if (abs(theVelocity) > MIN_CENTER)
       analogWrite(itsDrive, 0);                        
    else
    {
       analogWrite(itsDrive, 0);                          // stop
       return false;
    };

    //Serial.println("Moved\n");
    
    return true;                                          // action

  }

   
};

// -------------------------------------------------------------------------------------------------------
// Pan / Title Motors - defines Move() and constructor that defines pins and ranges (% PWM)
//--------------------------------------------------------------------------------------------------------

class PanMotor : public PwmMotor
{
  public :
     PanMotor() : PwmMotor(leftPin, rightPin, 1, 30) {};
     bool Input()
     {
       int aVelocity = nunchuk.analogX - CENTER_POINT + X_CENTER;
       if (aVelocity < 0) aVelocity = map(abs(aVelocity), 1, RANGE_LEFT, 1, 100) * -1;
       else aVelocity = map(abs(aVelocity), 1, RANGE_RIGHT, 1, 100);
       return(Move(aVelocity));      
     }
} Pan;

class TiltMotor : public PwmMotor
{
  public :
     TiltMotor() : PwmMotor(downPin, upPin, 1, 30) {};
     bool Input()
     {
       int aVelocity = nunchuk.analogY - CENTER_POINT + Y_CENTER;
       if (aVelocity < 0) aVelocity = map(abs(aVelocity), 1, RANGE_DOWN, 1, 100) * -1;
       else aVelocity = map(abs(aVelocity), 1, RANGE_UP, 1, 100);
       return(Move(aVelocity));
     }
} Tilt;

// -------------------------------------------------------------------------------------------------------
// Serial input control
//--------------------------------------------------------------------------------------------------------
class SerialControl                                       // serial input control
{

  public:

  SerialControl()                                         // constructor
  {
   
  }

  bool Input()                                            // process input
  {

    bool anAction = false;

    if (Serial.available() > 0)                           // if we input available
    {
      anAction = true;                                    // we took action, or at least assume it..
      char aChar = Serial.read();                         // read character
      if (aChar != '\n')                                  // not a newline
      {  Serial.print("Processing "); Serial.println(aChar);}; // echo it
      unsigned long aSt = millis();                       // grab current time
      switch(aChar)                                       // test command
      {
       case '#' :                                         // got a #?
        while (Serial.available() < 4)                    // must have 4 characters - hex for command, hex for subcmd
          delay(10);
        byte aByte[2];                                    // bytes read
        for (int i = 0; i < 2; i ++)                      // read both bytes
        {
          aByte[i] = 0;                                   // assume 0
          
          for (int j = 0; j < 2; j++)                     // read 2 nibbles
          {
            byte aB = Serial.read();                      // read a nibble
            if (aB >= '0' && aB <= '9')                   // convert nibble from text 
               aB -= '0';
            else
            if (aB >= 'a' && aB <= 'f')
               aB -= 'a' - 10;
            else
               aB -= 'A' - 10;
             aByte[i] |= aB;                              // add nibble
             if (j == 0)                                  // first nibble?
                aByte[i] <<= 4;                           // shift to prepare for next nibble
          }

        }               
        
        Serial.print("Issue command "); Serial.print((unsigned int) aByte[0],HEX); Serial.print(" - "); Serial.println((unsigned int) aByte[1], HEX);
        Camera.SendCode(aByte[0], aByte[1]);
        
        break;
       case 'I' :                                         // toggle display
        Camera.ToggleDisplay();
        break;
       case 'l' :
        Camera.Left();
        break;
      case 'r' :
        Camera.Right();
        break;
      case 'u' :
        Camera.Up();
        break;
      case 'd' :
        Camera.Down();
        break;
      case 's' :
        Camera.Select();
        break;
      case 'm' :
        Camera.Menu();
        break;                                        
       case 'U' :                                         // move up
        for (int i=0; i< 50; i++)Tilt.Move(100); delay(1000);                                 
        break;
       case 'D' :                                         // move down
        for (int i=0; i< 50; i++)Tilt.Move(-100); delay(1000);
        break;
       case 'L' :                                         // move left
        for (int i=0; i< 50; i++)Pan.Move(-100); delay(1000);
        break;
       case 'R' :                                         // move right
        for (int i=0; i< 50; i++)Pan.Move(100); delay(1000); 
        break;
       case 'P' :                                         // power on
        if (!Camera.IsPowered()) Camera.PowerOn();        
        break;
       case 'S' :                                         // sleep
        if (Camera.IsPowered()) Camera.PowerOff();        
        Serial.println("Sleep has been sent");
        break;
       case '-' :                                         // zoom out
        while (millis() - aSt < 1000)
           Camera.Zoom(-3);
        break;
       case '+' :                                         // zoom in
        while (millis() - aSt < 1000)
           Camera.Zoom(3);
        break;
       //default:
      }

    }

    return(anAction);
    
  }
  
} SerialCtrl;

//--------------------------------------------------------------------------------------------------------
// Set things up
//--------------------------------------------------------------------------------------------------------
void setup()
{
  
Serial.begin(115200);                                       // Opening the serial port

Serial.println("Initializing nunchuck");

nunchuk.init();                                             // Initialize the Nunchuck code

nunchuk.update();

Serial.print("Nunchuk (");                                  // Assuming nobody is touching it, 
Serial.print(nunchuk.analogX);                              // lets us see what it reads at neutral
Serial.print(",");
Serial.print(nunchuk.analogY);
Serial.println(")");

Pan.Setup();                                                // setup pan
Tilt.Setup();                                               // setup tilt

// Setup PWM for pan/tilt pins
TCCR1A = _BV(COM1A1) | _BV(WGM10);
TCCR1B =  _BV(CS12) | _BV(CS10); // 0x05

TCCR2A = _BV(COM2A1) | _BV(WGM20);
TCCR2B =  _BV(CS22)  | _BV(CS21) | _BV(CS20);

Camera.Setup();


Serial.println("We are ready");

}

//--------------------------------------------------------------------------------------------------------
// The main loop
//--------------------------------------------------------------------------------------------------------
void loop()
{
  
static unsigned long aLastAct = millis();                               // last activity time
static unsigned long aLastCheck = millis();                             // last camera check
static int aMode = 0;                                                   // mode 0 - normal (1 - menu control)

bool  anAction = false;                                                 // no action yet
bool  cDown = cButton.Down();                                           // get pre-update state
bool  zDown = zButton.Down();                                           // get pre-update state

nunchuk.update();                                                      // update nunchuk data

//Serial.print("Nunchuk (");                                  // Assuming nobody is touching it, 
//Serial.print(nunchuk.analogX);                              // lets us see what it reads at neutral
//Serial.print(",");
//Serial.print(nunchuk.analogY);
//Serial.println(")");
//nunchuk.analogX = 130; nunchuk.analogY = 130;

unsigned long aCurTime = millis();                                      // current time

anAction |= zButton.Input();                                            // get z button state
anAction |= cButton.Input();                                            // get c button state

if (cDown & !cButton.Down() && zButton.Down())                          // changed state while other button is down as well
   zButton.itsDownTime = aCurTime;                                      // reset z button down time (keep from bobbling releasing from both down)

if (zDown & !zButton.Down() && cButton.Down())                          // changed state while other button is down as well
   cButton.itsDownTime = aCurTime;                                      // reset c button down time (keep from bobbling releasing from both down)

anAction |= SerialCtrl.Input();                                         // get serial input

static unsigned long aLoopCnt = 0;
aLoopCnt ++;
if (aCurTime - aLastCheck > 5000)                                       // get status
{
  //Serial.println(aLoopCnt);
  aLoopCnt = 0;
  aLastCheck = aCurTime;
  Camera.Display();
  
//  Serial.print("accelX: "); Serial.println(nunchuk.accelX);
//  Serial.print("accelY: "); Serial.println(nunchuk.accelY);
//  Serial.print("accelZ: "); Serial.println(nunchuk.accelZ);
  
};

//
// Holding both buttons down more than 5 seconds toggles power.
//

if (zButton.Down() && cButton.Down())                                   // both buttons down?

{                                                                       // begin both buttons down

  Tilt.Stop();                                                          // stop - user doing something special
  Pan.Stop();                                                           // stop - user doing something special
  
  if (aCurTime - zButton.itsDownTime  > POWEROFF_TIME &&                 // both buttons down long enough?
      aCurTime - cButton.itsDownTime  > POWEROFF_TIME)
  {
    
     if (Camera.IsPowered())                                            // is camera on?
     {
      Pan.Stop();                                                       // full stop!
      Tilt.Stop();                                                      // full stop!
      Serial.println("Power off");
      Camera.PowerOff();                                                // turn off camera
      while (cButton.Down() || Camera.IsPowered())                      // while button is down and still powered on
      
      {
         delay(1000);                                                   // wait for poweroff to work
         if (Camera.IsPowered()) Camera.PowerOff();                     // if still powered on, try again
         nunchuk.update();                                              // update state
         cButton.Input();                                               // check button again
      }
      if (!Camera.IsPowered())                                          // powered off?
         Serial.println("Power is off!");                               // say so
     }
     else
     {
        Serial.println("Power on");                             
        Camera.PowerOn();                                               // power up
        while (cButton.Down() || !Camera.IsPowered())                   // while button down but not powered on
        {
          delay(1000);
          if (!Camera.IsPowered()) Camera.PowerOn();                    // if camera still not on, try again
          nunchuk.update();                                             // update state
          cButton.Input();                                              // check button again
        }
        if (Camera.IsPowered())                                         // did we detect power?
           Serial.println("Power is on!");                              // say so
     }
     
     cButton.itsDownTime = aCurTime;                                    // reset timer
     zButton.itsDownTime - aCurTime;                                    // reset timer
     
  }

  else

  if (aCurTime - zButton.itsDownTime > MODE_TIME &&                     // both buttons down long enough?
      aCurTime - cButton.itsDownTime > MODE_TIME &&
      abs(nunchuk.analogY - CENTER_POINT + Y_CENTER) > 80)              // if stick up or down?

  {                                                                     // begin change modes

    if (nunchuk.analogY - CENTER_POINT + Y_CENTER > 0)                  // up?

    {                                                                   // begin change mode

        aMode = (aMode + 1) % 2;                                        // only 2 modes, switch
      
        Camera.ToggleDisplay();                                         // toggle display

        Serial.print("Mode is now "); Serial.println(aMode);
      
    }                                                                   // end change mode
 
    else                                                                // if down

    {                                                                   // begin toggle display

        Camera.ToggleDisplay();                                         // toggle display
    
    };                                                                  // end toggle display

    cButton.itsDownTime = aCurTime;                                     // reset timer
    zButton.itsDownTime - aCurTime;                                     // reset timer

  };                                                                    // end change mode
  
}                                                                       // end both buttons down

else

if (aMode == 0)                                                         // normal mode

{                                                                       // begin normal mode

  if (Camera.IsPowered())                                               // if camera is on (dangerous to tilt/pan with it off - someone might be physically handling camera)
  {
    anAction |= Tilt.Input();                                           // tilt
    anAction |= Pan.Input();                                            // pan
    anAction |= Zoom.Input();                                           // zoom
  }

}                                                                       // end normal mode

else

if (aMode == 1)                                                         // menu mode

{                                                                       // begin menu mode

   static unsigned long aLastCmd = millis();                            // get last command time

   if (aCurTime - aLastCmd > 250)                                       // if last command long enough ago

   {                                                                    // begin take action

      int aX = nunchuk.analogX - CENTER_POINT + X_CENTER;               // get X
      int aY = nunchuk.analogY - CENTER_POINT + Y_CENTER;               // get Y

      if (abs(aX) > 80)                                                 // intentionally moved

      {                                                                 // begin move left/right

          if (aX > 0)                                                   // moving right
             Camera.Right();                                            // move right
          else                                                          // otherwise
             Camera.Left();                                             // move left

          aLastCmd = aCurTime;                                          // had action
      
      }                                                                 // end move left/right

      else                                                              // otherwise
      
      if (abs(aY) > 80)                                                 // intentionally moved

      {                                                                 // begin move up/down

          if (aY > 0)                                                   // moving up?
             Camera.Up();                                               // move up
          else                                                          // otherwise
             Camera.Down();                                             // move down

          aLastCmd = aCurTime;                                          // had action
      
      }                                                                 // end move up/down

      else                                                              // otherwise

      if (zButton.Down() && cButton.Down())                             // both buttons down?

      {                                                                 // begin ignore
         
      }                                                                 // end ignore

      else                                                              // otherwise

      if (zButton.Down() && aCurTime - zButton.itsDownTime > 100)       // c button is definitely down

      {

          Camera.Select();                                              // being select

          aLastCmd = aCurTime;                                          // mark action

      }
      else 

      if (cButton.Down() && aCurTime - cButton.itsDownTime > 100)       // c button is definitely down

      {

          Camera.Menu();                                                // being menu

          aLastCmd = aCurTime;                                          // mark action

      }; 

   };                                                                   // end take action

};                                                                      // end menu mode

if (!Camera.IsPowered())                                                 // camera not powered?
{                                                                       // begin stop tilt/pan
  Tilt.Stop();
  Pan.Stop();                                                           
}                                                                       // end stop tilt/pan

if (anAction)                                                           // had action?
   aLastAct = aCurTime;                                                 // store last action timestamp

if (Camera.IsPowered() && (((aCurTime - aLastAct) / 1000 / 60) >= INACTIVE_TIMEOUT)) // inactive?

{                                                                       // begin power off

  Serial.println("Turning off camera.");

  Camera.PowerOff();                                                    // power off
  delay(5000);                                                          // wait for poweroff to work
    
};                                                                      // end power off

}
