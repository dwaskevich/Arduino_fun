// Date and time functions using a DS3231 RTC connected via I2C and Wire lib
#include "RTClib.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// define MBR3 slave address
#define MBR3_I2C_SLAVE_ADDRESS 0x08
#define STATUS_REGISTER_ADDRESS 0xb0


RTC_DS3231 rtc;

char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

void setup () {
  Serial.begin(9600);

  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    abort();
  }

  // initiate the Wire library and join I2C bus as Master (no parameter needed for Master)
  Wire.begin();

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  if (rtc.lostPower()) {
    Serial.println("RTC lost power, let's set the time!");
    // When time needs to be set on a new device, or after a power loss, the
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }

  // When time needs to be re-set on a previously configured device, the
  // following line sets the RTC to the date & time this sketch was compiled
  // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  // This line sets the RTC with an explicit date & time, for example to set
  // January 21, 2014 at 3am you would call:
  // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));

  // Clear the buffer
  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
  display.drawFastHLine(9, 0, 90, SSD1306_WHITE);
  display.setCursor(9, 1);
  display.println(F(" MBR3 gestures "));
  display.display(); // Show initial text
  display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
  display.setCursor(0, 12);
}

uint8_t centroid, previousCentroid = 0;
char str [80];
unsigned long previousMillis;

void loop () {

  if(millis() - previousMillis >= 5000)
  {
    previousMillis = millis();
  
    DateTime now = rtc.now();
    Serial.println();
    Serial.print(now.year(), DEC);
    Serial.print('/');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.day(), DEC);
    Serial.print(" (");
    Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
    Serial.print(") ");
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.println();

    Serial.print("Temperature: ");
    Serial.print(rtc.getTemperature());
    Serial.println(" C");
    Serial.println();
  }

  delay(5);

  /* Get the touch position (centroid) of the slider */
  Wire.requestFrom(MBR3_I2C_SLAVE_ADDRESS, 1); // attempt to get MBR3 attention
  Wire.beginTransmission(MBR3_I2C_SLAVE_ADDRESS); // transmit to device
  Wire.write(STATUS_REGISTER_ADDRESS);  // sends address of MBR3 status register      
  Wire.endTransmission(false);  // don't send STOP, leave bus transaction open for requesting byte(s)
  
  Wire.requestFrom(MBR3_I2C_SLAVE_ADDRESS, 1);    // just request one byte for now (slider status) ... will send STOP
  while(Wire.available()) // Now retrieve the data
  {
    centroid = Wire.read(); // receive a byte as character
  }

  if(centroid != previousCentroid)
  {
    Serial.println(centroid, HEX);
    previousCentroid = centroid;

//      display.setCursor(0, 12);
//      sprintf(str, "     ");
//      display.print(str);
//      display.display();
    
    display.setCursor(0, 12);
    sprintf(str, "0x%02X\r\n", centroid);
    display.print(str);
    display.display();
  }    
}
