// Robust Rotary encoder reading
//
// Copyright John Main - best-microcontroller-projects.com
//

#include <LiquidCrystal.h>

#define BUTTON PB9
#define LED PC13

//LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
LiquidCrystal lcd(PA9, PA8, PB15, PB14, PB13, PB12);
//LiquidCrystal lcd(PA2, PA3, PA4, PA5, PA6, PA7);

void setup() {
  pinMode(BUTTON, INPUT);
  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(LED, OUTPUT);
  Serial.begin (9600);

  lcd.begin(16,2);

//  delay(500);
  
  lcd.print("Rotary Encoder");
  lcd.setCursor(0, 1);
  
}

void loop() {
  static int counter = 0;
  if(0 == digitalRead(BUTTON))
  {
    lcd.setCursor(0, 1);
    lcd.print("          ");
    lcd.setCursor(0, 1);
    lcd.print(counter);
    digitalWrite(LED, LOW);
    counter++;
    while(0 == digitalRead(BUTTON))
      ;
  }
  else
    digitalWrite(LED, HIGH);
}
