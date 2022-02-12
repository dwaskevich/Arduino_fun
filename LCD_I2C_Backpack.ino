/*
 * Author:  David Waskevich
 * Date:    28-Dec-2020
 * Name:    LCD_I2C_Backpack
 * 
 * Description: Using I2C LCD Backpack library from https://www.mathertel.de/Arduino/LiquidCrystal_PCF8574.aspx
 * 
 */

#include <LiquidCrystal_PCF8574.h>
#include <Wire.h>

LiquidCrystal_PCF8574 lcd(0x27); // set the LCD address to 0x27 for a 16 chars and 2 line display

void setup() {
  // put your setup code here, to run once:
  lcd.begin(16, 2); // initialize LCD

  lcd.setBacklight(255);
  lcd.home();
  lcd.clear();
  lcd.print("Hello LCD ... xx");
  lcd.setCursor(0, 1);
  lcd.print("Second line 1234");

}

void loop() {
  // put your main code here, to run repeatedly:
  lcd.setCursor(12, 1);
  lcd.print("    ");
  lcd.setCursor(12, 1);
  lcd.print(millis()/1000);
  delay(1000);

}
