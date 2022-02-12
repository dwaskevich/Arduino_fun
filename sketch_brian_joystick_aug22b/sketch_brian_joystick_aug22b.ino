int SW_pin = 2;
const int X_pin = 0;
const int Y_pin = 1;
int U_LED = 6;
int D_LED = 5;
int L_LED = 11;
int R_LED = 10;
int SW_LED = LED_BUILTIN; // was 4 in Brian's sketch

#define PRINT_DEBUG_MESSAGES

#define HYSTERISIS (3u)
#define SAMPLES_TO_AVERAGE (16u)
#define DEAD_ZONE (2u)

int X_Val_Accumulated = 0;
int Y_Val_Accumulated = 0;
int X_Val;
int Y_Val;
int X_Val_Previous = 0;
int Y_Val_Previous = 0;
int X_Mid_Point;
int Y_Mid_Point;

void setup() {
  pinMode(SW_pin, INPUT);
  pinMode(U_LED, OUTPUT);
  pinMode(D_LED, OUTPUT);
  pinMode(L_LED, OUTPUT);
  pinMode(R_LED, OUTPUT);
  pinMode(SW_LED, OUTPUT);
  digitalWrite(SW_pin, HIGH);
  Serial.begin(9600);
  Serial.print("djjw - Win10 - Application started ...\r\n");
  delay(500);

  for(int i = 0; i < SAMPLES_TO_AVERAGE; i++){
    X_Val_Accumulated += analogRead(X_pin);
    Y_Val_Accumulated += analogRead(Y_pin);
  }

  X_Mid_Point = X_Val_Accumulated/SAMPLES_TO_AVERAGE;
  Y_Mid_Point = Y_Val_Accumulated/SAMPLES_TO_AVERAGE;

  Serial.print("X accumulated = ");
  Serial.print(X_Val_Accumulated);
  Serial.print("   X average/mid point = ");
  Serial.print(X_Mid_Point);
  Serial.print("    Y accumulated = ");
  Serial.print(Y_Val_Accumulated);
  Serial.print("   Y average/midpoint = ");
  Serial.print(Y_Mid_Point);
  Serial.print("\n\n");
}


void loop() {
  X_Val = analogRead(X_pin);
  Y_Val = analogRead(Y_pin);
  
  if(X_Val < X_Mid_Point - DEAD_ZONE){
    analogWrite(L_LED, map(X_Val, 0, X_Mid_Point, 255, 0));
    analogWrite(R_LED, 0);
  }
  else if (X_Val > X_Mid_Point + DEAD_ZONE) {
    analogWrite(R_LED, map(X_Val, X_Mid_Point, 1023, 0, 255));
    analogWrite(L_LED, 0);
  }
  else {
    digitalWrite(R_LED, LOW);
    digitalWrite(L_LED, LOW);
  }
  
  if(Y_Val < Y_Mid_Point - DEAD_ZONE){
    analogWrite(D_LED, map(Y_Val, 0, Y_Mid_Point, 255, 0));
    analogWrite(U_LED, 0);
  }
  else if (Y_Val > Y_Mid_Point + DEAD_ZONE) {
    analogWrite(U_LED, map(Y_Val, Y_Mid_Point, 1023, 0, 255));
    analogWrite(D_LED, 0);
  }
  else {
    digitalWrite(D_LED, LOW);
    digitalWrite(U_LED, LOW);
  }

  if(digitalRead(SW_pin) == LOW) {
    digitalWrite(SW_LED, HIGH);
  }
  else {
    digitalWrite(SW_LED, LOW);
  }

#ifdef PRINT_DEBUG_MESSAGES

  if(X_Val > HYSTERISIS && (X_Val < (X_Val_Previous - HYSTERISIS) || X_Val > (X_Val_Previous + HYSTERISIS))){
    Serial.print("X Val: ");
    Serial.print(X_Val);
    Serial.print("\n");
    X_Val_Previous = X_Val;
  }

  if(Y_Val > HYSTERISIS && (Y_Val < (Y_Val_Previous - HYSTERISIS) || Y_Val > (Y_Val_Previous + HYSTERISIS))){
    Serial.print("Y Val: ");
    Serial.print(Y_Val);
    Serial.print("\n");
    Y_Val_Previous = Y_Val;
  }

#endif
    
}
