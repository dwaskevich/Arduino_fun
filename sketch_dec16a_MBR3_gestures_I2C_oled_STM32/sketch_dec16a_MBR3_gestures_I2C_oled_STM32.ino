/*
  DJJW - MBR3 Host sample application

  Use I2C interface to talk to MBR3 eval kit shield
  
  The circuit:
  - Use I2C on shield header to communicate with MBR3 shield.
  - 
  
  created 16-Dec-2020
  by David Waskevich

  Updates:
    17-Dec-2020 - included header files from Cypress MBR3 Host API that define MBR3 registers
                - updated register references to symbolic names from Host API header
                - updated "dump" routine to handle more than 32 bytes
    17-Dec-2020 - further updated "dump" routine to take user input from serial monitor
                - included MBR3 Host API register definitions (CY8CMBR3xxx_Device.h and CY8CMBR3xxx_Registers.h)
    7-Jan-2021  - added slider gestures (merging PSoC4 code)
                - changed MBR3 device selection in CY8CMBR3xxx_Device.h- #define CY8CMBR3xxx_DEVICE (CY8CMBR3xxx_CY8CMBR3106S)
                - Note - using CY8CKIT-042 kit to mimic CY8CMBR3106S
    9-Jan-2021  - added I2C OLED display (see STM32_testing_9-Jan-2021.ino)
                  - Note - had to downgrade Adafruit_GFX library to 1.7.2 (see https://www.stm32duino.com/viewtopic.php?t=474)
                  - Note - need pullups on I2C lines
    10-Jan-2021 - added CLICK_HOLD_DRAG to gesture types
                  added NEW_DISPLAY_TIME_TO_LIVE items to main loop
                  added speed bar to OLED display
                  updated swipe thresholds
                    - displacement = 25, time = 100 ms
                  added PWM output (Blue LED)
    
*/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306_STM32.h>
#include "CY8CMBR3xxx_Device.h"
#include "CY8CMBR3xxx_Registers.h"

/* Define tri-color LED pin assignments (can be driven with digital I/O or PWM for brightness control) */
#define RED PB1
#define GND PB0
#define GREEN PA7
#define BLUE PA6

#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

// define MBR3 slave address
#define MBR3_I2C_SLAVE_ADDRESS 0x08

// define status register address of interest for "s" command - 0xaa = button status, 0xb0 = slider 1 position
//#define STATUS_REGISTER_ADDRESS CY8CMBR3xxx_BUTTON_STAT
#define STATUS_REGISTER_ADDRESS CY8CMBR3xxx_SLIDER1_POSITION

#define MAIN_LOOP_DELAY_MSEC 1u

#define OLED_CENTROID_Y_POSITION 12u
#define OLED_GESTURE_Y_LOCATION 23u

/*****************************************************************************
* MACRO Definitions
*****************************************************************************/     

/* Set macro to '1' to enable centroid logging, '0' to disable logging */
#define ENABLE_CENTROID_LOGGING     (1u)

/* Finite state machine states for gesture decode (djjw)
    WAIT_FOR_TOUCH - wait for centroid value to be different than #define SLIDER_NO_TOUCH
    DECODE_GESTURE - finger event detected, decode gesture (look for centroid movement >= GESTURE_POSITION_THRESHOLD)
    WAIT_FOR_LIFTOFF - no temporal component to gesture algorithm at this point ... just wait for liftoff */
typedef enum
{
    WAIT_FOR_TOUCH = 0x01u,
    DECODE_GESTURE = 0x02u,
    WAIT_FOR_LIFTOFF = 0x03u
} GESTURE_DECODE_STATE;

/* Gesture decode types (djjw)
    NO_GESTURE - what it sounds like
    TOUCHDOWN_GESTURE - finger event detected, waiting for next event
    CLICK_GESTURE - occurs if lift-off happens with no centroid displacement within MAX_TOUCH_DURATION_FOR_CLICK
    SWIPE_GESTURE_FAST_RIGHT - swipe gesture determined by centroid displacement and elapsed time
    SWIPE_GESTURE_SLOW_RIGHT - swipe gesture determined by centroid displacement and elapsed time
    SWIPE_GESTURE_FAST_LEFT - swipe gesture determined by centroid displacement and elapsed time
    SWIPE_GESTURE_FAST_LEFT - swipe gesture determined by centroid displacement and elapsed time 
    CLICK_HOLD_DRAG - allows user to pause on slider and then track centroid movement */
typedef enum
{
  NO_GESTURE,
  TOUCHDOWN,
  CLICK_GESTURE,
  SWIPE_GESTURE_FAST_RIGHT,
  SWIPE_GESTURE_SLOW_RIGHT,
  SWIPE_GESTURE_FAST_LEFT,
  SWIPE_GESTURE_SLOW_LEFT,
  CLICK_HOLD_DRAG
} GESTURE_TYPE;

// Displacement threshold for swipe gestures (units are in centroid values)
#define GESTURE_DISPLACEMENT_THRESHOLD 25u

// Temporal gesture settings (units are msec)
#define MIN_TOUCH_DURATION 10u
#define MAX_TOUCH_DURATION_FOR_CLICK 150u
#define MAX_TOUCH_DURATION_FOR_SWIPE 250u
#define SWIPE_FAST_SLOW_THRESHOLD 100u

#define SLIDER_RESOLUTION 100u
#define SLIDER_NO_TOUCH 0xFFu
#define CLICK_INCREMENT 5u
#define SPEED_INCREMENT (SLIDER_RESOLUTION / 8)
#define MAX_SPEED_BAR_VALUE 126u

#define NEW_DISPLAY_TIME_TO_LIVE 750u

#define INCLUDE_CENTROID_ON_OLED (0u)

/*****************************************************************************
* Global Variables
*****************************************************************************/ 
volatile unsigned long gestureTimeStamp = 0;
bool newDisplayUpdateFlag = false;
unsigned long newDisplayTimeToLive;
bool newDisplayCentroidFlag = false;
uint8_t centroid;
bool clickHoldDragFlag = false;
int speedBarValue, previousSpeedBarValue = 0;
char str [80];

/*****************************************************************************
* Function Prototypes
*****************************************************************************/ 
void Decode_Gestures(void);
void printDisplacement(uint32_t, uint32_t, GESTURE_TYPE);

const int ledPin =  LED_BUILTIN; // the number of the LED pin

void setup() {
  // start serial communications
  Serial.begin(9600);
  Serial.print("\r\nMBR3 Host Application started ...\r\n");
  
  pinMode(ledPin, OUTPUT); // set the digital pin as output

  pinMode(GND, OUTPUT);
  digitalWrite(GND, LOW);
  
  pinMode(RED, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(BLUE, OUTPUT);

  // initiate the Wire library and join I2C bus as Master (no parameter needed for Master)
  Wire.begin();  

  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

  // Clear the buffer
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(BLACK, WHITE);
  display.drawFastHLine(9, 0, 96, WHITE);
  display.setCursor(9, 1);
  display.println(F(" STM32 gestures "));
  display.display(); // Show initial text
  display.setTextColor(WHITE, BLACK);
  display.setCursor(0, OLED_GESTURE_Y_LOCATION);

  /*  drawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color); 
      drawRoundRect(int16_t x0, int16_t y0, int16_t w, int16_t h,
                     int16_t radius, uint16_t color);
  */
  display.drawRoundRect(0, 48, 128, 15, 3, WHITE);
  display.display();
  
  delay(1000);
  Serial.print("\r\nMBR3 Host Application started ...\r\n");
}

void loop() {
  Decode_Gestures();
  delay(MAIN_LOOP_DELAY_MSEC);

  if(true == newDisplayUpdateFlag)
  {
    /* clear display information if TIME_TO_LIVE is expired */
    if(millis() - newDisplayTimeToLive >= NEW_DISPLAY_TIME_TO_LIVE)
    {
      display.setCursor(0, OLED_GESTURE_Y_LOCATION);
      display.print("               ");
      display.display();
      newDisplayUpdateFlag = false;
    }    
  }

  if(true == clickHoldDragFlag) /* print centroid tracking information to OLED display */
  {
    speedBarValue = centroid;
    sprintf(str, "0x%02X", centroid);
    display.setCursor(0, OLED_CENTROID_Y_POSITION);
    display.print(str);
    display.display();
  }

  if(speedBarValue != previousSpeedBarValue)
  {
    previousSpeedBarValue = speedBarValue;
    display.fillRoundRect(1, 49, 126, 13, 2, BLACK);
    display.fillRoundRect(1, 49, (speedBarValue * 126)/100, 13, 2, WHITE);
    display.display();
    analogWrite(BLUE, (speedBarValue * 255)/MAX_SPEED_BAR_VALUE);
  }

  #if INCLUDE_CENTROID_ON_OLED
    if(true == newDisplayCentroidFlag)
    {
      sprintf(str, "0x%02X", centroid);
      display.setCursor(0, OLED_CENTROID_Y_POSITION);
      display.print(str);
      display.display();
      newDisplayCentroidFlag = false;
    }
  #endif /* INCLUDE_CENTROID_ON_OLED */
}

/*******************************************************************************
* Function Name: Decode_Gestures
********************************************************************************
* Summary:
* In this function, flick gestures are decoded by interpreting linear slider
* centroid displacements (GESTURE_DISPLACEMENT_THRESHOLD).
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void Decode_Gestures()
{
  #if ENABLE_CENTROID_LOGGING
    // djjw - adding a print log for diagnostics
    #define FALSE (0u)
    #define TRUE (1u)
    #define MAX_LOG_ENTRIES 200u
    static uint16_t centroidLog[MAX_LOG_ENTRIES][2];
    static uint16_t logIndex = 0;
    static uint16_t gestureTriggeredIndex;
    static char touchIsActive = FALSE;
    static uint8_t gestureEvent;
    uint16_t i;
  #endif /* ENABLE_CENTROID_LOGGING */

  uint8_t previousCentroid;
  static GESTURE_DECODE_STATE currentState = WAIT_FOR_TOUCH;
  static uint8_t touchdownCentroidValue;
  static GESTURE_TYPE gesture = NO_GESTURE;
  static uint8_t previousGesture = NO_GESTURE;
      
  /* Get the touch position (centroid) of the slider */
  Wire.beginTransmission(MBR3_I2C_SLAVE_ADDRESS); // transmit to device
  Wire.write(STATUS_REGISTER_ADDRESS);  // sends address of MBR3 status register      
  Wire.endTransmission(false);  // don't send STOP, leave bus transaction open for requesting byte(s)
    
  Wire.requestFrom(MBR3_I2C_SLAVE_ADDRESS, 1);    // just request one byte for now (slider status) ... will send STOP
  while(Wire.available()) // Now retrieve the data
  {
    centroid = Wire.read(); // receive a byte as character
    if(centroid != previousCentroid)
    {
      newDisplayCentroidFlag = true;
      previousCentroid = centroid;
    }
  }

  /* process gesture state machine */
  switch( currentState )
  {
    case  WAIT_FOR_TOUCH:
        if(SLIDER_NO_TOUCH != centroid)
        {
          touchdownCentroidValue = centroid; /* record touchdown value */
          gestureTimeStamp = millis(); /* record touchdown time stamp */
          gesture = TOUCHDOWN; /* post gesture */
          currentState = DECODE_GESTURE; /* move to next state */
        }
        break;
    
    case  DECODE_GESTURE:
        if(SLIDER_NO_TOUCH == centroid) /* liftoff event happened (i.e. NO_TOUCH detected) */
        {
//          if(CLICK_HOLD_DRAG == gesture)
//          {
//            gesture = NO_GESTURE;
//            clickHoldDragFlag = false;
//          }
          
          if((millis() - gestureTimeStamp) < MIN_TOUCH_DURATION || (millis() - gestureTimeStamp) > MAX_TOUCH_DURATION_FOR_CLICK)
          {
            gesture = NO_GESTURE; /* abandon gesture if lift-off occurs quicker than MIN_TOUCH_DURATION or lasts longer than MAX */
            clickHoldDragFlag = false;
            #if ENABLE_CENTROID_LOGGING
              gestureTriggeredIndex = logIndex; /* record scan index for log dump later */
              gestureEvent = gesture;
            #endif /* ENABLE_CENTROID_LOGGING */
          }                    
          else
          {
            gesture = CLICK_GESTURE; /* click occurs if lift-off happens with no centroid displacement */
            #if ENABLE_CENTROID_LOGGING
              gestureTriggeredIndex = logIndex; /* record scan index for log dump later */
              gestureEvent = gesture;
            #endif /* ENABLE_CENTROID_LOGGING */
          }
          currentState = WAIT_FOR_TOUCH; /* go back to WAIT_FOR_TOUCH state */
        }
        else /* liftoff hasn't happened yet ... still in active gesture decode */
        {
          /* test for right swipe conditions (i.e. has there been enough displacement) */
          if(touchdownCentroidValue < (SLIDER_RESOLUTION - GESTURE_DISPLACEMENT_THRESHOLD) && \
            centroid > (touchdownCentroidValue + GESTURE_DISPLACEMENT_THRESHOLD))
          {
            /* now test for speed */
            if((millis() - gestureTimeStamp) < SWIPE_FAST_SLOW_THRESHOLD)
            {
              gesture = SWIPE_GESTURE_FAST_RIGHT;
              #if ENABLE_CENTROID_LOGGING
                gestureTriggeredIndex = logIndex; /* record scan index for log dump later */
                gestureEvent = gesture;
              #endif /* ENABLE_CENTROID_LOGGING */
            }
            else
              if((millis() - gestureTimeStamp) < MAX_TOUCH_DURATION_FOR_SWIPE)
              {
                gesture = SWIPE_GESTURE_SLOW_RIGHT;
                #if ENABLE_CENTROID_LOGGING
                gestureTriggeredIndex = logIndex; /* record scan index for log dump later */
                gestureEvent = gesture;
              #endif /* ENABLE_CENTROID_LOGGING */
              }
                       
            currentState = WAIT_FOR_LIFTOFF;            
          }

          /* test for left swipe conditions (i.e. has there been enough displacement) */
          if(touchdownCentroidValue > (0 + GESTURE_DISPLACEMENT_THRESHOLD)  && \
            centroid < (touchdownCentroidValue - GESTURE_DISPLACEMENT_THRESHOLD))
          { 
            /* now test for speed */
            if((millis() - gestureTimeStamp) < SWIPE_FAST_SLOW_THRESHOLD)
            {
              gesture = SWIPE_GESTURE_FAST_LEFT;
              #if ENABLE_CENTROID_LOGGING
                gestureTriggeredIndex = logIndex; /* record scan index for log dump later */
                gestureEvent = gesture;
              #endif /* ENABLE_CENTROID_LOGGING */
            }
            else
              if((millis() - gestureTimeStamp) < MAX_TOUCH_DURATION_FOR_SWIPE)
              {
                gesture = SWIPE_GESTURE_SLOW_LEFT;
                #if ENABLE_CENTROID_LOGGING
                  gestureTriggeredIndex = logIndex; /* record scan index for log dump later */
                  gestureEvent = gesture;
                #endif /* ENABLE_CENTROID_LOGGING */
              }
            
            currentState = WAIT_FOR_LIFTOFF;
            
          }

          if((millis() - gestureTimeStamp) >= MAX_TOUCH_DURATION_FOR_SWIPE) /* activate CLICK_HOLD_DRAG */
          {
            gesture = CLICK_HOLD_DRAG;
            #if ENABLE_CENTROID_LOGGING
              gestureTriggeredIndex = logIndex; /* record scan index for log dump later */
              gestureEvent = gesture;
            #endif /* ENABLE_CENTROID_LOGGING */

            currentState = WAIT_FOR_LIFTOFF;
          }
          
        }
        break;
        
    case  WAIT_FOR_LIFTOFF:
        newDisplayTimeToLive = millis();
        if(SLIDER_NO_TOUCH == centroid)
        {
          gesture = NO_GESTURE;
          currentState = WAIT_FOR_TOUCH;
        }
        break;
    
    default:
        break;
  }

  /* Place gesture actions here */
  if(gesture != previousGesture)
  {
    previousGesture = gesture;
    switch(gesture)
    {
      case NO_GESTURE:
          digitalWrite(ledPin, HIGH);
          display.setCursor(0, OLED_CENTROID_Y_POSITION);
          display.print("     ");
          display.display();
          break;
      case TOUCHDOWN:
          digitalWrite(ledPin, LOW);
          break;
      case CLICK_GESTURE:
          Serial.println("CLICK");
          display.setCursor(0, OLED_GESTURE_Y_LOCATION);
          display.print("               ");
          display.setCursor(0, OLED_GESTURE_Y_LOCATION);
          display.print("CLICK");
          display.display();
          digitalWrite(ledPin, LOW);
          speedBarValue += CLICK_INCREMENT;
          if(speedBarValue >= MAX_SPEED_BAR_VALUE)
            speedBarValue = MAX_SPEED_BAR_VALUE;
          newDisplayUpdateFlag = true;
          newDisplayTimeToLive = millis();
          break;
      case SWIPE_GESTURE_FAST_RIGHT:
          sprintf(str, "FAST SWIPE RIGHT %d (touchdown value = %d, time = %d msec)\r\n", (centroid - touchdownCentroidValue), touchdownCentroidValue, (millis() - gestureTimeStamp));
          Serial.print(str);
          display.setCursor(0, OLED_GESTURE_Y_LOCATION);
          display.print("               ");
          display.setCursor(0, OLED_GESTURE_Y_LOCATION);
          display.print("FAST_RIGHT");
          display.display();
          printDisplacement(touchdownCentroidValue, centroid, gesture);
          digitalWrite(ledPin, LOW);
          speedBarValue += SPEED_INCREMENT * 2;
          if(speedBarValue >= MAX_SPEED_BAR_VALUE)
            speedBarValue = MAX_SPEED_BAR_VALUE;
          newDisplayUpdateFlag = true;
          newDisplayTimeToLive = millis();
          break;
      case SWIPE_GESTURE_SLOW_RIGHT:
          sprintf(str, "SLOW SWIPE RIGHT %d (touchdown value = %d, time = %d msec)\r\n", (centroid - touchdownCentroidValue), touchdownCentroidValue, (millis() - gestureTimeStamp));
          Serial.print(str);
          display.setCursor(0, OLED_GESTURE_Y_LOCATION);
          display.print("               ");
          display.setCursor(0, OLED_GESTURE_Y_LOCATION);
          display.print("SLOW_RIGHT");
          display.display();
          printDisplacement(touchdownCentroidValue, centroid, gesture);
          digitalWrite(ledPin, LOW);
          speedBarValue += SPEED_INCREMENT;
          if(speedBarValue >= MAX_SPEED_BAR_VALUE)
            speedBarValue = MAX_SPEED_BAR_VALUE;
          newDisplayUpdateFlag = true;
          newDisplayTimeToLive = millis();
          break;
      case SWIPE_GESTURE_FAST_LEFT:
          sprintf(str, "FAST SWIPE LEFT %d (touchdown value = %d, time = %d msec)\r\n", (touchdownCentroidValue - centroid), touchdownCentroidValue, (millis() - gestureTimeStamp));
          Serial.print(str);
          display.setCursor(0, OLED_GESTURE_Y_LOCATION);
          display.print("               ");
          display.setCursor(0, OLED_GESTURE_Y_LOCATION);
          display.print("FAST_LEFT");
          display.display();
          printDisplacement(touchdownCentroidValue, centroid, gesture);
          digitalWrite(ledPin, LOW);
          speedBarValue -= SPEED_INCREMENT * 2;
          if(speedBarValue <= 0)
            speedBarValue = 0;
          newDisplayUpdateFlag = true;
          newDisplayTimeToLive = millis();
          break;
      case SWIPE_GESTURE_SLOW_LEFT:
          sprintf(str, "SLOW SWIPE LEFT %d (touchdown value = %d, time = %d msec)\r\n", (touchdownCentroidValue - centroid), touchdownCentroidValue, (millis() - gestureTimeStamp));
          Serial.print(str);
          display.setCursor(0, OLED_GESTURE_Y_LOCATION);
          display.print("               ");
          display.setCursor(0, OLED_GESTURE_Y_LOCATION);
          display.print("SLOW_LEFT");
          display.display();
          printDisplacement(touchdownCentroidValue, centroid, gesture);
          digitalWrite(ledPin, LOW);
          speedBarValue -= SPEED_INCREMENT;
          if(speedBarValue <= 0)
            speedBarValue = 0;
          newDisplayUpdateFlag = true;
          newDisplayTimeToLive = millis();
          break;
      case CLICK_HOLD_DRAG:
          clickHoldDragFlag = true;
          sprintf(str, "CLICK_HOLD_DRAG %d (touchdown value = %d, time = %d msec)\r\n", (touchdownCentroidValue - centroid), touchdownCentroidValue, (millis() - gestureTimeStamp));
          Serial.print(str);
          display.setCursor(0, OLED_GESTURE_Y_LOCATION);
          display.print("               ");
          display.setCursor(0, OLED_GESTURE_Y_LOCATION);
          display.print("CLICK_HOLD_DRAG");
          display.display();
          printDisplacement(touchdownCentroidValue, centroid, gesture);
          digitalWrite(ledPin, LOW);
          speedBarValue = centroid;
          newDisplayUpdateFlag = true;
          newDisplayTimeToLive = millis();
          break;
      default:
          break;
      }
  }
  
  if(WAIT_FOR_TOUCH == currentState)
  {
    digitalWrite(ledPin, HIGH);
    clickHoldDragFlag = false;
  }

  #if ENABLE_CENTROID_LOGGING
    /* djjw - 19-Nov-2018 ... adding centroid logging for diagnostics */
    if(WAIT_FOR_TOUCH != currentState )
    {
      touchIsActive = TRUE; /* touch is active, start logging centroids */
      
      centroidLog[logIndex][0] = centroid;
      centroidLog[logIndex][1] = millis() - gestureTimeStamp;
      
      logIndex++;
      if(logIndex > MAX_LOG_ENTRIES)
        logIndex = MAX_LOG_ENTRIES;   
    }
    else if(TRUE == touchIsActive) /* this marks an end to the currently active touch event ... print log to UART */
    {
      sprintf(str, "Centroid Log - gesture triggerd at index #%d\n\rIndex\tCentroid\tTime(ms)\n\r", gestureTriggeredIndex);
      // UART_UartPutString(str);
      Serial.print(str);
      for(i = 0; i <= logIndex; i++)
      {
        sprintf(str, "%d\t%d\t\t%d", i, centroidLog[i][0], centroidLog[i][1]);
        // UART_UartPutString(str);
        Serial.print(str);
        if(i == gestureTriggeredIndex)
          Serial.println(" *** triggered ***");
        else
          Serial.println(" ");
      }
      sprintf(str, "End of log - touch was active for %d scans\n\r", logIndex);
      Serial.print(str);
      sprintf(str, "Touch event was %d\r\n\n", gestureEvent);
      Serial.print(str);
      touchIsActive = FALSE;
    }
    else /* this is the default/idle "no touch" state ... just keep resetting the Log index counter */
      logIndex = 0;
  #endif /* ENABLE_CENTROID_LOGGING */
    
}

#define MAXLENGTH 80
void printDisplacement(uint8_t touchdownCentroidValue, uint8_t centroid, GESTURE_TYPE gesture)
{
    char buf [MAXLENGTH];
    if(gesture == SWIPE_GESTURE_FAST_LEFT || gesture == SWIPE_GESTURE_FAST_RIGHT || gesture == SWIPE_GESTURE_SLOW_LEFT || gesture == SWIPE_GESTURE_SLOW_RIGHT)
    {
      if(centroid > touchdownCentroidValue)
      {
        snprintf(buf, MAXLENGTH, "Displacement = %d, Centroid at trigger = %d, Touchdown = %d \r\n\n", (centroid - touchdownCentroidValue), centroid, touchdownCentroidValue);
      }
      else
      {
        snprintf(buf, MAXLENGTH, "Displacement = %d, Centroid at trigger = %d, Touchdown = %d \r\n\n", (touchdownCentroidValue - centroid), centroid, touchdownCentroidValue);
      }
      Serial.print(buf);
    }
}


/* [] END OF FILE */
