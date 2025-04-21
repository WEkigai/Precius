/*------------------------------------
Precius Precision Cooktop
Developed by WEkigai B.V. 
Released under GLPv3. See included license file on Github. Libraries used may have been licensed under other licenses.

Know more: https://wekigai.eu/precius
Main project page https://github.com/WEkigai/Precius 

------------------------------------------*/

// Include libraries
#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <ArduPID.h>
#include <math.h>
#include <Encoder.h>
#include <Button2.h>

// Define all hardware pinouts

// Inputs


/// Buttons
#define BUTTON_DOWN_PIN 36
Button2 button_down(BUTTON_DOWN_PIN, true);

#define BUTTON_RIGHT_PIN 37
Button2 button_right(BUTTON_RIGHT_PIN, true);

#define BUTTON_MID_PIN 38 
Button2 button_mid(BUTTON_MID_PIN, true);

#define BUTTON_LEFT_PIN 39
Button2 button_left(BUTTON_LEFT_PIN, true);

#define BUTTON_UP_PIN 40
Button2 button_up(BUTTON_UP_PIN, true);

/// Encoder Knob

#define ENC_A 15
#define ENC_B 7
#define BUTTON_ENC_PIN 48  // Encoder button
Button2 button_enc(BUTTON_ENC_PIN,true);

// Encoder variables
volatile int encoderValue = 0;
volatile int lastStateA = 0;
int lastEncoderValue = -999;

/// Temperature sensors
//// Bottom sensor
#define BASE_SENSOR_PIN 5
//// Probe sensor
#define PROBE_SENSOR_PIN 6
// Outputs
/// Heater output realy (using LED pin for easy debug)
#define RELAY_PIN 41 //Relay on pin 41

//Buzzer
#define BUZZER_PIN 35  // Buzzer on pin 35

//LED
#define LED_PIN 10     // LED on pin 10


// Display

// TFT Display Pins (ST7789)
#define TFT_MOSI 11
#define TFT_SCLK 12
#define TFT_CS 21
#define TFT_DC 14
#define TFT_RST 47
#define TFT_BL 13

// Create the ST7789 TFT object using hardware SPI:
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);


// Temperature variables
double Tset = -1.0;           //The target temperature setpoint
double Tnow = -1.0;           // The current temperature
double T_base = -1.0;        //Temperature of the bottom sensor
double T_probe = -1.0;         // Temperature of the probe sensor
double dual_mode_buffer = 0;  //The difference in dual mode between probe and bottom sensors

// Power variables
float powerPercent = 50.0;  //Heater power compared to maximum possible power

// Time variables
unsigned long int loopTime;  // The wall clock time at the start of each loop


// Variables for timer
int timer_hours = 0;
int timer_minutes = 10;
int timer_seconds = 0;
unsigned int timer_refresh_period = 1000;  //How often will we decrement the timer (ms)
static boolean flag_timer_refresh = false;
unsigned long timer_refresh_StartTime = 0;

// State/mode variables

enum sensorModes {
  BOTTOM_SENSOR_ONLY,  //Only bottom sensor is used to get actual temperature, probe sensor is ignored
  PROBE_SENSOR_ONLY,   //Only probe sensor is used to get actual temperature, bottom sensor is ignored
  DUAL_MODE,            //Both sensors are used. Actual temperature = max(probe, bottom-dual_mode_buffer)
  MANUL_MODE            //Temperature sensors are ignored and heater power is set by user
};
enum sensorModes sensorMode=BOTTOM_SENSOR_ONLY;



enum timerModes {
  TARGET_AND_HOLD,  //Once target is reached, hold the target indefinitely
  TARGET_AND_STOP,  // Once the target is reached, stop heating
  TARGET_AND_TIMER  // Once the target is reached, start a timer and stop after timer ends
};
timerModes timerMode=TARGET_AND_HOLD;

enum heaterStates {
  HEATER_OFF,  // Heater is powered off (default)
  HEATER_ON    // Heater is powered on
};
heaterStates heaterState=HEATER_OFF;

enum tempUnits { //Temperature units
  UNIT_C,  // Degree Celcius
  UNIT_F   // Degree Farenheit
};
tempUnits tempUnit=UNIT_C;


enum screens { //States for different screens to display
  HOME_SCREEN,  // Home Screen
  TIME_SETING_SCREEN, //Time setting
  SETTINGS_SCREEN   // Settings screen
};
screens screen=HOME_SCREEN;

// Control variables
// Control type = PID
// PID parameters

ArduPID myPID;  // PID object
double control_output = 0.0;


// Parameters for converting PID output to PWM
unsigned int duty_windowSize = 5000;  //total duty cycle of 5 seconds
unsigned long duty_windowStartTime;   //start time of each duty cycle window
unsigned int duty_cycle = 0;          //number between 0 and duty_windowSize that defines duty cycle

// Hardware parameters
float Vref = 3.3;              // Reference voltage [V]
float analog_levels = 4095.0;  //12 bit resolution of ESP32 analog read

// NTC parameters for base sensor
float R_ref_base = 10000;  // Reference Resistor t [ohm]
float R_0_base = 100000;   // value of rct in T_0 [ohm]
float T_0_base = 298.15;   // use T_0 in Kelvin [K]
float Vout_base = 0.0;     // Vout
float Rout_base = 0.0;     // Rout 
// use the datasheet to get this data.
float beta_base = 3990;  // initial parameters [K]
float TempK_base = 0.0;  // variable output

// NTC parameters for probe sensor
float R_ref_probe = 10000;  // Reference Resistor t [ohm]
float R_0_probe = 100000;   // value of rct in T_0 [ohm]
float T_0_probe = 298.15;   // use T_0 in Kelvin [K]
float Vout_probe = 0.0;     // Vout
float Rout_probe = 0.0;     // Rout
// use the datasheet to get this data.
float beta_probe = 3990;  // initial parameters [K]
float TempK_probe = 0.0;  // variable output



//Parameters to refresh display
unsigned int display_refresh_period = 1000;  //milliseconds between display updates
unsigned long display_refresh_StartTime;    //start time of each refresh cycle
static boolean flag_display_refresh = true;  //display refresh flag


//Function prototypes
void do_readTemp();      //Function to handle temperature readings
void do_display();       // Function to handle display
void do_heater();        // Function to handle heater
void do_statemachine();  // Handles all states; sets variables to right value depending on state
void do_timer();         //Handles the timer countdown
void do_control();       // Handles the control algorithm

void up_button_click(Button2& btn);
void down_button_click(Button2& btn);
void left_button_click(Button2& btn);
void right_button_click(Button2& btn);
void mid_button_click(Button2& btn);
void enc_button_click(Button2& btn);

// Interrupt service routine for the rotary encoder
void IRAM_ATTR readEncoder() {
  int stateA = digitalRead(ENC_A);
  int stateB = digitalRead(ENC_B);
  if (stateA != lastStateA) {
    encoderValue += (stateB != stateA) ? 1 : -1;
  }
  lastStateA = stateA;
}

void setup() 
{

Serial.begin(921600);
Serial.println("Started Serial");

  // Initiate all hardware

  ///Buttons
button_up.begin(BUTTON_UP_PIN,INPUT_PULLUP,true);
button_down.begin(BUTTON_DOWN_PIN,INPUT_PULLUP,true);
button_left.begin(BUTTON_LEFT_PIN,INPUT_PULLUP,true);
button_right.begin(BUTTON_RIGHT_PIN,INPUT_PULLUP,true);
button_mid.begin(BUTTON_MID_PIN,INPUT_PULLUP,true);
button_enc.begin(BUTTON_ENC_PIN,INPUT_PULLUP,true);

button_up.setClickHandler(up_button_click);
button_down.setClickHandler(down_button_click);
button_left.setClickHandler(left_button_click);
button_right.setClickHandler(right_button_click);
button_mid.setClickHandler(mid_button_click);
button_enc.setClickHandler(enc_button_click);



// Encoder
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  lastStateA = digitalRead(ENC_A);
  attachInterrupt(digitalPinToInterrupt(ENC_A), readEncoder, CHANGE);


// Display
  SPI.begin(TFT_SCLK, -1, TFT_MOSI, TFT_CS);

  tft.init(240, 320);
  tft.setRotation(3);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(4);

  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

  tft.setCursor(40, 128);
  tft.print("WELCOME");
  delay(2000);
  tft.fillScreen(ST77XX_BLACK);


// LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

// Relay Pin
  pinMode(RELAY_PIN, OUTPUT);    // SSR is an output now
  digitalWrite(RELAY_PIN, LOW);  // Start with SSR OFF


  // Initiate times
  // setup PID duty cycle window start time
  duty_windowStartTime = millis();
  //setup display refresh period
  display_refresh_StartTime = millis();

  // Initiate control
  myPID.begin(&Tnow, &control_output, &Tset, 100, 1.0, 10.0);
  myPID.setOutputLimits(0, duty_windowSize * powerPercent * 0.01);
  myPID.setWindUpLimits(0, 1000);
  myPID.start();
}

void loop() 
{

  loopTime = millis();  // Get loop start time

  // Read temperatures
  do_readTemp();

  // Read buttons
  button_up.loop();
  button_down.loop();
  button_left.loop();
  button_right.loop();
  button_mid.loop();
  button_enc.loop();


//Handle encoder changes


if (encoderValue != lastEncoderValue) {
    switch (screen){
      case HOME_SCREEN: {
        Tset += (encoderValue-lastEncoderValue);
        if(Tset<10)Tset=10;
        if(tempUnit==UNIT_C && Tset>260)Tset=260;
        if(tempUnit==UNIT_F && Tset>550)Tset=500;
        break;
      }
      case TIME_SETING_SCREEN: {
        break;
      }
      case SETTINGS_SCREEN: {
        break;
      }

    }
//Finally store current encoder value for next loop
    lastEncoderValue = encoderValue;
  }

  // Update states and other variables

  // State machine
  do_statemachine();

  // Perform temperature control
  do_control();


// Update all timers at the end of the loop (heater, display, timer) and set flags to refresh
  //check relay window time
  if (loopTime - duty_windowStartTime > duty_windowSize) {
    //time to shift the Relay Window
    duty_windowStartTime += duty_windowSize;
  }

  if (loopTime - display_refresh_StartTime > display_refresh_period) {
    display_refresh_StartTime += display_refresh_period;
    flag_display_refresh = true;
  }

  if (loopTime - timer_refresh_StartTime > timer_refresh_period) {
    timer_refresh_StartTime = +timer_refresh_period;
    flag_timer_refresh = true;
  }

  // Update display
  do_display();

  // Update heater
  do_heater();
}
/* End of loop*/


void do_readTemp() 
{

// Read temperature of base

Vout_base=float(analogReadMilliVolts(BASE_SENSOR_PIN))/1000.0;
Rout_base=R_ref_base*(Vref/Vout_base - 1.0);
TempK_base=1.0/(((log(Rout_base/R_0_base))/beta_base)+(1/(T_0_base)));

if(tempUnit==UNIT_C)T_base=TempK_base-273.15;
if(tempUnit==UNIT_F)T_base=((TempK_base-273.15)*5.0/9.0)-32.0;

Serial.println(Rout_base);

// Read temperature of probe

Vout_probe=float(analogReadMilliVolts(PROBE_SENSOR_PIN))/1000.0;
Rout_probe=R_ref_probe*(Vref/Vout_probe - 1.0);
TempK_probe=1.0/(((log(Rout_probe/R_0_probe))/beta_probe)+(1/(T_0_probe)));

if(tempUnit==UNIT_C)T_probe=TempK_probe-273.15;
if(tempUnit==UNIT_F)T_probe=((TempK_probe-273.15)*5.0/9.0)-32.0;
}

void do_statemachine()
{
  switch (sensorMode) {
    case BOTTOM_SENSOR_ONLY:
      Tnow = T_base;
      break;

    case PROBE_SENSOR_ONLY:
      Tnow = T_probe;
      break;

    case DUAL_MODE:
      Tnow = max(T_probe, T_base - dual_mode_buffer);
      break;
 }

  switch (timerMode) {
    case TARGET_AND_HOLD:
      break;

    case TARGET_AND_STOP:
      if (Tnow >= Tset) heaterState = HEATER_OFF;  //If we have exceeded the target temperature, stop heating
      break;

    case TARGET_AND_TIMER:
      do_timer();
      if (timer_hours == 0 && timer_minutes == 0 && timer_seconds == 0) heaterState = HEATER_OFF;  //If the timer is done, turn the heater off
      break;
  }
}

void do_control(){
  if(sensorMode!=MANUL_MODE){ //we are in modes that require control
  //perform PID calculations
  myPID.setOutputLimits(0, duty_windowSize*powerPercent*0.01);
  myPID.setWindUpLimits(0,2*Tset);//Seeting windup limits based on target temperature. Higher setpoints need higher I term to stabilize temperature due to ambient losses
  myPID.compute();
  duty_cycle=control_output;

  }
  else // We are in manual mode
  {
    duty_cycle=duty_windowSize*powerPercent*0.01;
  }
}


void do_display() {
//if it is time to update display update it and set the refresh flag back to false
  if (flag_display_refresh) {

if(screen==HOME_SCREEN){
// Clear display
//tft.fillScreen(ST77XX_BLACK);
tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
tft.setTextSize(2);

// Display Mode
tft.setCursor(10, 10);
tft.print("Mode:");
tft.setCursor(100, 10);
//Serial.println(screen);
tft.print("               ");
tft.setCursor(100, 10);
if(sensorMode==BOTTOM_SENSOR_ONLY)tft.print("Base only");
if(sensorMode==PROBE_SENSOR_ONLY)tft.print("Probe only");
if(sensorMode==DUAL_MODE)tft.print("Dual Sensor");
if(sensorMode==MANUL_MODE)tft.print("Manual");

// Display Power

tft.setCursor(10, 30);
tft.print("Power:");
tft.setCursor(100, 30);
tft.print(powerPercent);

// Display Target Temperature

tft.setCursor(10, 50);
tft.print("Tset:");
tft.setCursor(100, 50);
tft.print(Tset);

// Display bottom sensor temperature
tft.setCursor(10, 70);
tft.print("Tbase:");
tft.setCursor(100, 70);
tft.print(T_base);

// Display probe sensor temperature
tft.setCursor(10, 90);
tft.print("Tprobe:");
tft.setCursor(100, 90);
tft.print(T_probe);

//Display heater state
tft.setCursor(10, 110);
tft.print("Heater:");
tft.setCursor(100, 110);
tft.print(heaterState);
// Display timer 
}

if(screen==TIME_SETING_SCREEN){
tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
tft.setTextSize(2);

// Display Mode
tft.setCursor(10, 10);
tft.print("HH:");
tft.setCursor(10, 30);
tft.print("MM:");
tft.setCursor(10, 50);
tft.print("SS");

tft.setCursor(80, 10);
tft.print(timer_hours);
tft.setCursor(80, 30);
tft.print(timer_minutes);
tft.setCursor(80, 50);
tft.print(timer_seconds);

    }
  }
}

void do_heater() {
  if ((duty_cycle >= loopTime - duty_windowStartTime) && (heaterState == HEATER_ON)) {
    digitalWrite(RELAY_PIN, HIGH);
    // Serial.println("   Relay On");
  } else {
    digitalWrite(RELAY_PIN, LOW);
    //Serial.println("   Relay Off");
  }
}

void do_timer() {
  if (flag_timer_refresh) {

    if (timer_seconds > 0) {
      timer_seconds--;
    } else {
      if (timer_minutes > 0) {
        timer_minutes--;
        timer_seconds = 59;
      } else {
        if (timer_hours > 0) {
          timer_hours--;
          timer_minutes = 59;
          timer_seconds = 59;
        }
      }
    }
  }
}

void up_button_click(Button2& btn){
  switch (screen){
    case HOME_SCREEN:{ //On home screen, up will increment power percent
      powerPercent++;
      if(powerPercent>100)powerPercent=100;
      break;
    }
  }
}

void down_button_click(Button2& btn){

  switch (screen){
    case HOME_SCREEN:{ //On home screen, down will decrement power percent
      powerPercent--;
      if(powerPercent<0)powerPercent=0;
      break;
    }
  }
}

void left_button_click(Button2& btn){

  switch (screen){
    case HOME_SCREEN:{ //On home screen, left will change modes
      if(sensorMode==BOTTOM_SENSOR_ONLY){
        sensorMode=MANUL_MODE;
        break;
      }
      if(sensorMode==MANUL_MODE){
        sensorMode=DUAL_MODE;
        break;
      }
      if(sensorMode==DUAL_MODE){
        sensorMode=PROBE_SENSOR_ONLY;
        break;
      }
      if(sensorMode==PROBE_SENSOR_ONLY){
        sensorMode= BOTTOM_SENSOR_ONLY;
        break;
      }
      break;
    }
  }
}

void right_button_click(Button2& btn){

  switch (screen){
    case HOME_SCREEN:{ //On home screen, right will change modes
      if(sensorMode==BOTTOM_SENSOR_ONLY){
        sensorMode=PROBE_SENSOR_ONLY;
        break;
        }
      if(sensorMode==PROBE_SENSOR_ONLY){
        sensorMode=DUAL_MODE;
        break;
      }
      if(sensorMode==DUAL_MODE){
        sensorMode=MANUL_MODE;
        break;
      }
      if(sensorMode==MANUL_MODE){
        sensorMode= BOTTOM_SENSOR_ONLY;
        break;
      }
      break;
    }
  }
}

void mid_button_click(Button2& btn){

  switch (screen){
    case HOME_SCREEN:{ //On home screen, mid button will start/stop heating
      if(heaterState==HEATER_OFF){
        heaterState=HEATER_ON;
        break;
      }
      if(heaterState==HEATER_ON){
        heaterState=HEATER_OFF;
        break;
      }
    break;
    }
  }
}

void enc_button_click(Button2& btn){
tft.fillScreen(ST77XX_BLACK);
if(screen==HOME_SCREEN){
  screen=TIME_SETING_SCREEN;
  return;
  }
if(screen==TIME_SETING_SCREEN){
  screen=HOME_SCREEN;
  return;
  }  
}
