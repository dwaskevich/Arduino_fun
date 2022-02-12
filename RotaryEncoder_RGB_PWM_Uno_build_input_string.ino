/*
 * Author:  David Waskevich
 * Date:    31-Dec-2020
 * Name:    RotaryEncoder_RGB_PWM_Uno_build_input_string
 * 
 * Description: Demo snippets of:
 *              - Serial communications via USB-to-UART
 *              - Rotary Encoder + button
 *              - Potentiometer (analog in)
 *              - PWM (tri-color LED)
 *              - Added String class to parse terminal input
 *              
 * Update - 11-Jan-2021:
 *              - replaced serialEvent() with just basic input string construction
 *                - Note - did this because serialEvent is not supported on STM32 Blue Pill
 */

// Robust Rotary encoder reading
//
// Copyright John Main - best-microcontroller-projects.com
//

#include <LiquidCrystal.h>

/* Define Rotary Encoder inputs (Clock, Data, Push Button) */
#define CLK 7
#define DATA 6
#define BUTTON 0

#define LED LED_BUILTIN
#define POT 0

/* Define tri-color LED pin assignments (can be driven with digital I/O or PWM for brightness control) */
#define RED 11
//#define GND
#define GREEN 10
#define BLUE 9

typedef enum States {
  Red_LED,
  Green_LED,
  Blue_LED,
  MAX_NUM_STATES  
} States_t;

int currentState = Red_LED;

LiquidCrystal lcd(12, 8, 5, 4, 3, 2);
int columnPosition = 0;

String inputString = "";       // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete

void setup() {
  pinMode(CLK, INPUT);
  pinMode(CLK, INPUT_PULLUP);
  pinMode(DATA, INPUT);
  pinMode(DATA, INPUT_PULLUP);
  pinMode(BUTTON, INPUT);
  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(LED, OUTPUT);

//  pinMode(GND, OUTPUT);
//  digitalWrite(GND, LOW);
  
  pinMode(RED, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(BLUE, OUTPUT);
  
  Serial.begin (9600);
  // reserve 200 bytes for the inputString:
  inputString.reserve(10);
  Serial.println("DJJW Rotary Encoder + LCD + RGB PWM on Lenovo");

  lcd.begin(16,2);
  lcd.clear();
  lcd.print("Arduino UNO");
  delay(2000);
  lcd.clear();
  lcd.print("RED  GRN  BLU");
  lcd.setCursor(0, 1);
  lcd.print('0');

  lcd.cursor();
  lcd.blink();
}

static uint8_t prevNextCode = 0;
static uint16_t store=0;

void loop() {
static int8_t val;
static int c;
char incomingByte = ' ';
static int16_t redVal, greenVal, blueVal = 0;
static int analogIn, prevAnalogIn, numSamples = 0;

  if(Serial.available() > 0)
  {
    // get the new byte:
    incomingByte = Serial.read();
    
    // add it to the inputString:
    inputString += incomingByte;
    // if the incoming character is a newline, set a flag so the main loop can do something about it
    if (incomingByte == '\n')
    {
      Serial.print(inputString);
      stringComplete = true;
    }
  }
  
  if (true == stringComplete) {
    Serial.println(inputString);
    if('r' == inputString.charAt(0))
    {
      inputString.remove(0, 2);
      redVal = inputString.toInt();
      if(redVal >= 255)
        redVal = 255;
      if(redVal <= 0)
        redVal = 0;
      analogWrite(RED, redVal);
      lcd.setCursor(0, 1);
      lcd.print("   ");
      lcd.setCursor(0, 1);
      lcd.print(redVal);
      currentState = Red_LED;
    }
    if('g' == inputString.charAt(0))
    {
      inputString.remove(0, 2);
      greenVal = inputString.toInt();
      if(greenVal >= 255)
        greenVal = 255;
      if(greenVal <= 0)
        greenVal = 0;
      analogWrite(GREEN, greenVal);
      lcd.setCursor(5, 1);
      lcd.print("   ");
      lcd.setCursor(5, 1);
      lcd.print(greenVal);
      currentState = Green_LED;
    }
    if('b' == inputString.charAt(0))
    {
      inputString.remove(0, 2);
      blueVal = inputString.toInt();
      if(blueVal >= 255)
        blueVal = 255;
      if(blueVal <= 0)
        blueVal = 0;
      analogWrite(BLUE, blueVal);
      lcd.setCursor(10, 1);
      lcd.print("   ");
      lcd.setCursor(10, 1);
      lcd.print(blueVal);
      currentState = Blue_LED;
    }
    
    // clear the string:
    inputString = "";
    stringComplete = false;
    
  }

//  if(Serial.available() > 0)
//  {
//    incomingByte = Serial.read();
//    Serial.print(incomingByte);
//    Serial.print("Analog value = ");
//    Serial.println(analogIn);
//  }

  if( val=read_rotary() ) {
      c +=val;
      
    switch(currentState)
    {
      case  Red_LED:
        redVal += val;
        if(redVal < 0)
          redVal = 255;
        if(redVal > 255)
          redVal = 0;
        analogWrite(RED, redVal);
        lcd.setCursor(0, 1);
        lcd.print("   ");
        lcd.setCursor(0, 1);
        lcd.print(redVal);
        break;
  
      case  Green_LED:
        greenVal += val;
        if(greenVal < 0)
          greenVal = 255;
        if(greenVal > 255)
          greenVal = 0;
        analogWrite(GREEN, greenVal);
        lcd.setCursor(5, 1);
        lcd.print("   ");
        lcd.setCursor(5, 1);
        lcd.print(greenVal);
        break;
  
      case  Blue_LED:
        blueVal += val;
        if(blueVal < 0)
          blueVal = 255;
        if(blueVal > 255)
          blueVal = 0;
        analogWrite(BLUE, blueVal);
        lcd.setCursor(10, 1);
        lcd.print("   ");
        lcd.setCursor(10, 1);
        lcd.print(blueVal);
        break;
  
      default:
        break;
     }
     
     Serial.print(c);Serial.print(" ");
      
     if ( prevNextCode==0x0b) {
        Serial.print("eleven ");
        Serial.println(store,HEX);
     }

     if ( prevNextCode==0x07) {
        Serial.print("seven ");
        Serial.println(store,HEX);
     }

     Serial.print("Analog value = ");
     Serial.println(analogIn);
   }

   analogIn += analogRead(POT);
   numSamples++;
   if(numSamples >= 32)
   {
      analogIn = analogIn / 32;
      if(analogIn != prevAnalogIn)
     {
//        lcd.setCursor(12, 1);
//        lcd.print("    ");
//        lcd.setCursor(12, 1);
//        lcd.print(analogIn);
        prevAnalogIn = analogIn;
     }
      numSamples = 0;
   }
   

   if(0 == digitalRead(BUTTON))
   {
    delay(100);
    if(0 == digitalRead(BUTTON))
    {
      while(0 == digitalRead(BUTTON))
        digitalWrite(LED, HIGH);
      digitalWrite(LED, LOW);
      currentState++;
      if(currentState >= MAX_NUM_STATES)
        currentState = 0;
      switch(currentState)
     {
      case  Red_LED:
        lcd.setCursor(0, 1);
        lcd.print(redVal);
        break;
  
      case  Green_LED:
        lcd.setCursor(5, 1);
        lcd.print(greenVal);
        break;
  
      case  Blue_LED:
        lcd.setCursor(10, 1);
        lcd.print(blueVal);
        break;
  
      default:
        break;
     }
    }
   }
}

// A vald CW or CCW move returns 1, invalid returns 0.
int8_t read_rotary() {
  static int8_t rot_enc_table[] = {0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0};

  prevNextCode <<= 2;
  if (digitalRead(DATA)) prevNextCode |= 0x02;
  if (digitalRead(CLK)) prevNextCode |= 0x01;
  prevNextCode &= 0x0f;

   // If valid then store as 16 bit data.
   if  (rot_enc_table[prevNextCode] ) {
      store <<= 4;
      store |= prevNextCode;
      //if (store==0xd42b) return 1;
      //if (store==0xe817) return -1;
      if ((store&0xff)==0x2b) return -1;
      if ((store&0xff)==0x17) return 1;
   }
   return 0;
}

/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}
