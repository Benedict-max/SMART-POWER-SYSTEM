#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);
int ADC_a = A0; //Analog Input
int ADC_Y = A1;
int ADC_B = A2;
float VIn_a = 0.0; //Voltage In after voltage divider
float VInY = 0.0;
float VInB = 0.0;
float Voltage_a = 0.0;       //Actual voltage after calculation
float VoltageY = 0.0;
float VoltageB= 0.0;
float CalVal = 11.08433; //Voltage divider calibration value
float AC_LOW_VOLT = 0.0;
float AC_HIGH_VOLT = 0.0;
float AC_LOW_VOLTY = 0.0;
float AC_HIGH_VOLTY = 0.0;
float AC_LOW_VOLTB = 0.0;
float AC_HIGH_VOLTB = 0.0;
//Timing
unsigned long previousMillis = 0;
const long interval = 200;          //Interval to read voltages

// input pin declearation
const int redPhaseIn = 13;
const int yellowPhaseIn = 12;
const int bluePhaseIn = 11;

// output pin declearation
const int redPhaseOut = 10;
const int yellowPhaseOut = 9;
const int bluePhaseOut = 8;

// variable for reading input phase status. This is keepin all input at 0 state
bool redState = false;
bool yellowState = false;
bool blueState = false;
bool allPhaseOn = false;

void setup() {
pinMode(redPhaseIn, INPUT);
pinMode(yellowPhaseIn, INPUT);
pinMode(bluePhaseIn, INPUT);

pinMode(redPhaseOut, OUTPUT);
pinMode(yellowPhaseOut, OUTPUT);
pinMode(bluePhaseOut, OUTPUT);

pinMode(A0,INPUT);
pinMode(A1,INPUT);
pinMode(A2,INPUT);

//Init serial monitor. This will help display what is happening on the serial monitor
Serial.begin(9600);
  // initialize the LCD
  lcd.init();
  lcd.backlight(); 
  lcd.print("SMART SYSTEM!");
  delay(2000);
  lcd.setCursor(0,1);
  lcd.print("Initializing"); 
  delay(1000);
}

void loop() {
   unsigned long currentMillis = millis();
  //checking for the state of each input
  int redPhaseState = digitalRead(redPhaseIn);
  int yellowPhaseState = digitalRead(yellowPhaseIn);
  int bluePhaseState = digitalRead(bluePhaseIn);
  
    VIn_a = (ADC_a * 5.00) / 1024.00;     //Convert 10bit input to an actual voltage 
    Voltage_a = (VIn_a * CalVal)-0.10;
    AC_LOW_VOLT = (Voltage_a / 1.414 );
    AC_HIGH_VOLT = ( AC_LOW_VOLT * 18.333);

    VInY = (ADC_Y * 5.00) / 1024.00;     //Convert 10bit input to an actual voltage
    VoltageY = (VInY * CalVal)-0.10;
    AC_LOW_VOLTY = (VoltageY / 1.414 );
    AC_HIGH_VOLTY = ( AC_LOW_VOLTY * 18.333);
 
    VInB = (ADC_B * 5.00) / 1024.00;     //Convert 10bit input to an actual voltage 
    VoltageB = (VInB * CalVal)-0.10;
    AC_LOW_VOLTB = (VoltageB / 1.414 );
    AC_HIGH_VOLTB = ( AC_LOW_VOLTB * 18.333);
  
  if ( redPhaseState == HIGH && yellowPhaseState == HIGH && bluePhaseState == HIGH && currentMillis - previousMillis >= interval && allPhaseOn == false )
  {
    allPhaseOn = true;
    previousMillis = currentMillis;
    analogRead (A0);
    Serial.println("All Systems On");
    lcd.clear();
    lcd.setCursor (0,0); // set to line 1, char 0  
    lcd.print("SMART POWER SYS");

    delay (1000);
    lcd.setCursor (0,1); // set to line 1, char 0  
    lcd.print("All Systems ON!");
    digitalWrite ( redPhaseOut,HIGH);
    digitalWrite ( yellowPhaseOut,LOW);
    digitalWrite ( bluePhaseOut,LOW);
    delay (1000);
    lcd.clear();
    lcd.setCursor (0,0); // set to line 1, char 0  
    lcd.print("SOLAR IN USE");
    lcd.setCursor (0,1); // set to line 1, char 0  
    lcd.print(AC_HIGH_VOLT);
    lcd.setCursor(6,1);
    lcd.print("V input");
     delay (1000);
   }
   else{
    if ( redPhaseState == HIGH )
      {
    Serial.println("Solar is on");
    redState = true;
    analogRead (A0);
    lcd.clear();
    lcd.setCursor (0,0); // set to line 1, char 0  
    lcd.print("SMART POWER SYS");
    delay (1000);
    digitalWrite ( redPhaseOut,HIGH);
    digitalWrite ( yellowPhaseOut,LOW);
    digitalWrite ( bluePhaseOut,LOW);
    delay (1000);
    lcd.clear();
    lcd.setCursor (0,0); // set to line 1, char 0  
    lcd.print("SOLAR IN USE");
    lcd.setCursor (0,1); // set to line 1, char 0  
    lcd.print(AC_HIGH_VOLT);
    lcd.setCursor(6,1);
    lcd.print("V input");
   //delay (500);
      }
      else if ( yellowPhaseState == HIGH  )
      {
    yellowState = true;
    Serial.println("Turbine is on"); 
    analogRead (A1);
    lcd.clear();
    lcd.setCursor (0,0); // set to line 1, char 0  
    lcd.print("SMART POWER SYS");
    
    delay (1000);
    lcd.setCursor (0,1); // set to line 1, char 0  
    lcd.print("Turbine ON!");
    digitalWrite ( redPhaseOut,LOW);
    digitalWrite ( yellowPhaseOut,HIGH);
    digitalWrite ( bluePhaseOut,LOW);
    delay (1000);
    lcd.clear();
    lcd.setCursor (0,0); // set to line 1, char 0  
    lcd.print("TURBINE ON!");
    lcd.setCursor (0,1); // set to line 1, char 0  
    lcd.print(AC_HIGH_VOLTY);
    lcd.setCursor(6,1);
    lcd.print("V input");
     // delay (500);
     }
      else if ( bluePhaseState == HIGH)
      {
    blueState = true;
    Serial.println("MAINS is on"); 
    analogRead (A2);
    lcd.clear();
    lcd.setCursor (0,0); // set to line 1, char 0  
    lcd.print("SMART POWER SYS");
   
    delay (1000);
    lcd.setCursor (0,1); // set to line 1, char 0  
    lcd.print("Mains ON!");
    digitalWrite ( redPhaseOut,LOW);
    digitalWrite ( yellowPhaseOut,LOW);
    digitalWrite ( bluePhaseOut,HIGH);   
    delay (1000);
    lcd.clear();
    lcd.setCursor (0,0); // set to line 1, char 0  
    lcd.print("MAINS IS ON!");
    lcd.setCursor (0,1); // set to line 1, char 0  
    lcd.print(AC_HIGH_VOLTB);
    lcd.setCursor(6,1);
    lcd.print("V input");   
     // delay (500);    
      }
      else
      {
    allPhaseOn = false;
    Serial.println("All phases are off");
    lcd.clear();
    lcd.setCursor (0,0); // set to line 1, char 0  
    lcd.print("SMART POWER SYS");
    delay (1000);
    lcd.setCursor (0,1); // set to line 1, char 0  
    lcd.print("ALL PHASES OFF!");
    digitalWrite ( redPhaseOut,LOW);
    digitalWrite ( yellowPhaseOut,LOW);
    digitalWrite ( bluePhaseOut,LOW);
      }
      }
   }
