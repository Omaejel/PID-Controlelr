#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <PID_v2.h>

#define THERMISTORPIN A10// which analog pin to connect - why do you declare it here as a define? Does it need to be defined in the setup loop?
#define RESISTORREF 9975 // refference resistor actual value
#define THERMISTORNOMINAL 100000 //nominal thermistor value @ 25C
#define TEMPERATURENOMINAL 25  //nominal thermistor temp
#define BCOEFFICIENT 3950 //given by thermistor manufacturer

LiquidCrystal_I2C lcd(0x27, 20, 4);  // I2C LCD address to 0x27 for a 20chars  4line disp
const int ssrpin = 5;

//***************** PID SETTINGS *******************
//When process tuning, start with i=0, d=0, then adjust P>0 to get quick response
//with minimal oscillation about the setpoint. Then set i>0, (might be very small)
//to to help reduce the oscillation error futher. Then Adjust d>0 to reduce
//overshoot, while also maybe needing to adjust p to reduce heater lag.
double Kp = 190;  //proportional output modifier (harder and faster ramp up power)
double Ki = .1;  //integral term (adjusts the "viewing window" for the algorithm)
double Kd = 150; // derivative term (prevents overshoot but can lag heating) 
PID_v2 myPID(Kp, Ki, Kd, PID::Direct);
const int WindowSize = 1000; //I am not sure what this does if you use an ssr and it also might just change the integral variable, effecively
unsigned long windowStartTime;
double actualtemp = 0;
double tempset = 150; 

//****************** EVENT TIMING *******************
//Time= millisecs between events, Millis= the last time event occured
long tempSampleTime = 20, tempSampleMillis = 0; // temperature check
long displayUpdateTime = 500, displayUpdateMillis = 0; // draw display

void setup() 
{
  Serial.begin(9600);

  lcd.init();  // - initialize the lcd 
  lcd.backlight();  //turn on backlight 
  lcd.clear();
  delay(1000); // Testing [dan]

  pinMode(ssrpin, OUTPUT);
  windowStartTime = millis();
  myPID.SetOutputLimits(0, WindowSize);
  myPID.Start(actualtemp, 0, tempset);  
}

void loop() {
  unsigned long currentMillis = millis(); 
  if(currentMillis - tempSampleMillis >= tempSampleTime) // temperature sampling timer
  {
    tempSampleMillis = currentMillis; // save last time temp was sampled
    actualtemp = checkTemperature();
  }

  if(currentMillis - displayUpdateMillis >= displayUpdateTime) // display update timer
  {
    displayUpdateMillis = currentMillis; // save last time display was updated
    displayUpdate();  
  }

  const double output = myPID.Run(actualtemp); // calculate PID output
  int ssrpwm = map(output, 0, WindowSize, 0, 255); // convert PID output to PMW duty cylce
  // Serial.print("PWM Value = ");
  // Serial.print(ssrpwm);
  // Serial.print("\n");
  analogWrite(ssrpin, ssrpwm); // send PWM duty cycle to SSR
}


double checkTemperature()
{
  float sample = 0;
  for (int i = 0; i < 10; i++) // take 10 amples
  {
    sample += analogRead(THERMISTORPIN); // read assigned pin and add to sample
  }    
  sample = sample / 10; // take the average reading of all 10 samples
  sample = (1023 / sample) -1; // returns calculated resistance of thermistor?(I think it uses this to set the analog read range)
  sample = RESISTORREF / sample; //adjusts calculation for real measuered constants?
  double steinhart; // this calculation uses the Steinhart equation
  steinhart = sample / THERMISTORNOMINAL;           // (R/Ro)
  steinhart = log(steinhart);                       // ln(R/Ro)
  steinhart /= BCOEFFICIENT;                        // 1/B * ln(R/Ro)
  steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                      // Invert
  steinhart -= 273.15;                              // convert to C
  steinhart = (steinhart * 9 / 5) + 32;             // convert to F
  return steinhart;
}

void displayUpdate()
{
  // lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("   Temp Set:  ");
  lcd.setCursor(14,0);
  int temp = tempset; //turns to an int with no decimal
  lcd.print(temp);

  if (actualtemp < 100)
  {
    lcd.setCursor(0,1);
    lcd.print("  Actual:   ");
    lcd.setCursor(12,1);  
    lcd.print(actualtemp);
    lcd.print("F");  
  }

  else
  {
    lcd.setCursor(0,1);
    lcd.print("  Actual:  ");
    lcd.setCursor(11,1);  
    lcd.print(actualtemp);
    lcd.print("F");  
  }

}