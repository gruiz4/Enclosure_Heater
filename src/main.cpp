#include <SPI.h>
#include <U8g2lib.h>
#include <Wire.h> 
#include <ESP32RotaryEncoder.h> 
#include <FanController.h>

// #include <Adafruit_SHT31.h>

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <NTC_Thermistor.h> // For the NTC thermistor

// --- Pin Definitions ---
// Display
#define DISPLAY_CS_PIN 5
#define DISPLAY_RST_PIN U8X8_PIN_NONE // For U8g2, U8X8_PIN_NONE if not used

// Encoder
const int ENCODER_A_PIN  = 26; // CLK for ESP32RotaryEncoder
const int ENCODER_B_PIN  = 25; // DT for ESP32RotaryEncoder
const int ENCODER_SW_PIN = 27; // Switch / Button
const int ENCODER_STEPS_PER_DETENT = 4; // Pulses per physical click/detent

RotaryEncoder rotaryEncoder(ENCODER_A_PIN, ENCODER_B_PIN, 50, ENCODER_STEPS_PER_DETENT);


// NTC Thermistor Pin (ensure this is an ADC capable pin on your ESP32, e.g., GPIO34)
const int NTC_SENSOR_PIN = 34;
const int RelayPin = 32;

// --- NTC Thermistor Configuration ---
#define NTC_REFERENCE_RESISTANCE    4883  // Value of the series resistor in Ohms (e.g., 4.7kOhms or 10kOhms)
#define NTC_NOMINAL_RESISTANCE      114400 // Nominal resistance of the thermistor at nominal temperature (e.g., 100kOhms for a 3950 NTC)
#define NTC_NOMINAL_TEMPERATURE     25.7    // Nominal temperature for the thermistor in Celsius (e.g., 25 C)
#define NTC_B_VALUE                 3950  // Beta coefficient (B-value) of the thermistor
#define NTC_ESP32_ANALOG_RESOLUTION 4095  // ADC resolution for ESP32 (12-bit ADC, 0-4095)
#define NTC_ESP32_ADC_VREF_MV       3280  // ADC reference voltage in millivolts (e.g., 3300mV for 3.3V)

float targetTemperature = 0.0;
int   targetFanSpeed = 50;
float enclosureTempSHT30 = -99.9;
float enclosureHumiditySHT30 = -99.9;
float heaterTempNTC = -99.9;
int minTemp = 0;
int maxTemp = 50; //min and max enclosure temperature
int DHTPIN = 4;
#define DHTTYPE    DHT11     // DHT 11
// --- Sensor Objects ---
// Adafruit_SHT31 sht31 = Adafruit_SHT31(); //will use in final build, 
DHT_Unified dht(DHTPIN, DHTTYPE);
Thermistor* ntc_thermistor; // Pointer for the NTC thermistor object




const int FanTachPin = 35;
const int SENSOR_THRESHOLD = 1000;
const int PWM_PIN = 12;

FanController fan(FanTachPin, SENSOR_THRESHOLD, PWM_PIN);
// --- U8g2 Display Object ---
// U8G2_ST7920_128X64_F_HW_SPI u8g2(U8G2_R0, DISPLAY_CS_PIN, DISPLAY_RST_PIN);
// U8G2_ST7565_LX12864_F_3W_SW_SPI u8g2(U8G2_R0, DISPLAY_CS_PIN, DISPLAY_RST_PIN);
U8G2_ST7920_128X64_F_SW_SPI u8g2(U8G2_R0, 18, 23, 5, DISPLAY_RST_PIN);
// --- Display Configuration ---
#define CS_PIN 5

#ifndef RST_PIN
  #define RST_PIN U8X8_PIN_NONE // U8g2 macro for no reset pin
#endif



// Buffer for formatting strings to display
char displayBuffer[32];


const int NUM_MENU_ITEMS = 6;
const int LINE_HEIGHT = 10;
const int DISPLAY_WIDTH = 128;
const int TEXT_X_OFFSET = 2;
int text_Y_baselines[NUM_MENU_ITEMS];

int selectedLine = 0;
bool editingMode = false;
int editingLine = -1;
long encoderValueAtEditStart = 0;
float initialValueForEditingFloat = 0.0;
int initialValueForEditingInt = 0;

bool turnedRightFlag = false;
bool turnedLeftFlag = false;
bool buttonPressedFlag = false;
bool buttonWasPressedThisLoop = false;
bool heaterEnabled = false;

long lastTime;

int lastEncoderPos;

void turnedRight()
{
	Serial.println( "Right ->" );

	// Set this back to false so we can watch for the next move
	turnedRightFlag = false;
}

void turnedLeft()
{
	Serial.println( "<- Left" );

	// Set this back to false so we can watch for the next move
	turnedLeftFlag = false;
}

void knobCallback( long value )
{
	if( turnedRightFlag || turnedLeftFlag )
		return;

	switch( value )
	{
		case 1:
	  		turnedRightFlag = true;
		break;

		case -1:
	  		turnedLeftFlag = true;
		break;
	}

	rotaryEncoder.setEncoderValue(0);
}

void buttonCallback()
{
  detachInterrupt(ENCODER_SW_PIN);
	// Serial.printf( "boop! button was down for %lu ms\n", duration );
  if (buttonPressedFlag){
    buttonPressedFlag = false;
  }
  else{
    buttonPressedFlag = true;
  }
  rotaryEncoder.setBoundaries(1,3,false);
  // rotaryEncoder.setEncoderValue(1);//i think this is a good implementation - i disagree
  lastEncoderPos = 0;
  
}

void editValues(int currentline){
  switch (currentline) {
    case 1:
      editingMode = false;
      Serial.print("editValues(1) called. Current heaterEnabled before toggle: ");
      Serial.println(heaterEnabled ? "true" : "false");

      editingMode = false; // This is good, makes it a one-shot action
      if (heaterEnabled) {
          heaterEnabled = false;
          Serial.println("editValues(1): heaterEnabled changed to -> false");
      } else {
          heaterEnabled = true;
          Serial.println("editValues(1): heaterEnabled changed to -> true");
      }
      
      // editingMode = false;
      delay(10);
      rotaryEncoder.setEncoderValue(1);
      lastEncoderPos = rotaryEncoder.getEncoderValue();
      break;

    case 2:
      if (rotaryEncoder.getEncoderValue() == lastEncoderPos){
        break;
      }
      else if (rotaryEncoder.getEncoderValue() > lastEncoderPos){
        if (targetTemperature < maxTemp){
          targetTemperature += 1;
          
        }
        else{
          targetTemperature = maxTemp;
        }
      }
      else if (targetTemperature > minTemp){
        targetTemperature -= 1;

      }
      else{
        targetTemperature =0;
      }
      rotaryEncoder.setEncoderValue(0);
      lastEncoderPos = rotaryEncoder.getEncoderValue();
      break;
    case 3:
      if (rotaryEncoder.getEncoderValue() == lastEncoderPos){
        break;
      }
      else if (rotaryEncoder.getEncoderValue() > lastEncoderPos){
        if (targetFanSpeed < 100){
          targetFanSpeed += 5;
        }
        else{
          targetFanSpeed = 100;
        }
      }
      else if (targetFanSpeed > 0){
        targetFanSpeed -= 5;

      }
      else{
        targetFanSpeed=0;
      }
      rotaryEncoder.setEncoderValue(0);
      lastEncoderPos = rotaryEncoder.getEncoderValue();
      break;
      
      
  }
  
}

void readNTCSensor() {
  float heaterTemps[10];
  heaterTempNTC = 0;
  for (int i=0; i<9;i++){
   
    heaterTempNTC += ntc_thermistor->readCelsius();
  }
  heaterTempNTC = heaterTempNTC/10;
  if (isnan(heaterTempNTC)){
    heaterTempNTC = -99.9;
    Serial.println("Failed to read temperature from NTC");
  }
  
}

void readSHT30Sensor() { //modified for use with DHT11 instead of SHT30
    // enclosureTempSHT30 = sht31.readTemperature();
    // enclosureHumiditySHT30 = sht31.readHumidity();

    sensors_event_t event;
    dht.temperature().getEvent(&event); 
    enclosureTempSHT30 = event.temperature;
    if (! isnan(enclosureTempSHT30)) {  // check if 'is not a number'
      Serial.print("Temp *C = "); Serial.print(enclosureTempSHT30); Serial.print("\t\t");
    } else { 
      enclosureTempSHT30 = -99.0;
      Serial.println("Failed to read temperature");
    }
    
    dht.humidity().getEvent(&event);
    enclosureHumiditySHT30 = event.relative_humidity ;

    if (! isnan(enclosureHumiditySHT30)) {  // check if 'is not a number'
      Serial.print("Hum. % = "); Serial.println(enclosureHumiditySHT30);
    } 
    else { 
      enclosureHumiditySHT30 = -99.0;
      Serial.println("Failed to read humidity");
    }

}
void setup() {
  // Initialize Serial communication for debugging (optional)
  Serial.begin(115200);
  Serial.println("U8g2 ESP32 Display Test - Troubleshooting Build");

  SPI.begin(); // Assumes default pins for VSPI. U8g2 handles its own CS_PIN.

  // Initialize the U8g2 library
  u8g2.begin();

  u8g2.setBusClock(800000);
  Serial.println("Set SPI bus clock");

  // Set a font.
  u8g2.setFont(u8g2_font_helvR08_tf);
  
    int fontAscent = u8g2.getAscent();
    int fontMaxHeight = u8g2.getMaxCharHeight();
    int textOffsetY = fontAscent + (LINE_HEIGHT - fontMaxHeight) / 2;
    if (textOffsetY < fontAscent) textOffsetY = fontAscent;
    if (textOffsetY <= 0) textOffsetY = fontAscent;

    for (int i = 0; i < NUM_MENU_ITEMS; ++i) {
        text_Y_baselines[i] = (i * LINE_HEIGHT) + textOffsetY;
        if (text_Y_baselines[i] < fontAscent) text_Y_baselines[i] = fontAscent;
        if (text_Y_baselines[i] > (u8g2.getDisplayHeight() - (fontMaxHeight - fontAscent))) {
             text_Y_baselines[i] = u8g2.getDisplayHeight() - (fontMaxHeight - fontAscent);   
        }
    }
    

  Serial.println("Display initialized");
  // Wire.begin(); 
  // sht31.begin(0x44);

  dht.begin();
  fan.begin();

  ntc_thermistor = new NTC_Thermistor_ESP32(
        NTC_SENSOR_PIN,
        NTC_REFERENCE_RESISTANCE,
        NTC_NOMINAL_RESISTANCE,
        NTC_NOMINAL_TEMPERATURE,
        NTC_B_VALUE,
        NTC_ESP32_ADC_VREF_MV,
        NTC_ESP32_ANALOG_RESOLUTION
    );

  rotaryEncoder.setEncoderType(FLOATING);
  rotaryEncoder.setBoundaries(1,3,false);
  // rotaryEncoder.onTurned(&knobCallback);
  // rotaryEncoder.onPressed(buttonCallback);
  pinMode(ENCODER_SW_PIN, INPUT);
  attachInterrupt(ENCODER_SW_PIN, buttonCallback,  FALLING);
  rotaryEncoder.begin();

  pinMode(RelayPin, OUTPUT);


}

void loop(){
  
  // if (turnedRightFlag){

  // }

  if (buttonPressedFlag){
    delay(300);
    Serial.println("Pressed button");
    attachInterrupt(ENCODER_SW_PIN, buttonCallback,  FALLING);
    buttonPressedFlag = false;


    if (editingMode){
      rotaryEncoder.setBoundaries(1,3,true);
      rotaryEncoder.setEncoderValue(1);

      editingMode = false;
    }
    else{
      selectedLine = (int)rotaryEncoder.getEncoderValue();
      rotaryEncoder.setBoundaries(-100,100,true);
      rotaryEncoder.setEncoderValue(0);
      editingMode = true;
    }
    
    
    // if (!editingMode){
    //   if (selectedLine == 1){
    //     heaterEnabled = true;
    //   }
    //   else if (selectedLine == 2) // setting target temperature
    //   {
    //     rotaryEncoder.setBoundaries(0,50,true); //can set temperature from zero Celsius to 50 Celsius

    //     while (buttonPressedFlag == false){ // as long as button isn't pressed, can adjust temp, only refreshes that line on screen.
    //       sprintf(displayBuffer, "Target Temp: %.1f C", rotaryEncoder.getEncoderValue());
    //       u8g2.setDrawColor(1);
    //       u8g2.drawBox(0, 2 * LINE_HEIGHT, DISPLAY_WIDTH, LINE_HEIGHT); //2*LineHeight argument b/c second row
    //       u8g2.setDrawColor(0);
    //       u8g2.setCursor(TEXT_X_OFFSET, text_Y_baselines[2]);
    //       u8g2.print(displayBuffer);
    //       u8g2.setDrawColor(1);
    //     }
    //     rotaryEncoder.setBoundaries(1,3,false);
    //   }
    //   else{
    //     rotaryEncoder.setBoundaries(0,50,true); //can set temperature from zero Celsius to 50 Celsius

    //     while (buttonPressedFlag == false){ // as long as button isn't pressed, can adjust temp, only refreshes that line on screen.
    //       sprintf(displayBuffer, "Target Temp: %.1f C", rotaryEncoder.getEncoderValue());
    //       u8g2.setDrawColor(1);
    //       u8g2.drawBox(0, 2 * LINE_HEIGHT, DISPLAY_WIDTH, LINE_HEIGHT); //2*LineHeight argument b/c second row
    //       u8g2.setDrawColor(0);
    //       u8g2.setCursor(TEXT_X_OFFSET, text_Y_baselines[2]);
    //       u8g2.print(displayBuffer);
    //       u8g2.setDrawColor(1);
    //     }
    //     rotaryEncoder.setBoundaries(1,3,false);
      // }
      
    }

  if (!editingMode){
     selectedLine = (int)rotaryEncoder.getEncoderValue(); 
    }
  else{
      editValues(selectedLine);
    }
  // Serial.print("Loop Check -- Before Relay Logic -- ");
  // Serial.print("heaterEnabled: "); Serial.print(heaterEnabled ? "true" : "false");
  // Serial.print(" | SHT30 Temp: "); Serial.print(enclosureTempSHT30);
  // Serial.print(" | Target Temp: "); Serial.print(targetTemperature);
  // Serial.print(" | Humidity: "); Serial.println(enclosureHumiditySHT30);

  if (heaterEnabled && enclosureHumiditySHT30 > 0 && enclosureTempSHT30 < targetTemperature) {
    digitalWrite(RelayPin, HIGH);
    // Serial.println("RELAY DECISION: --- HIGH ---");
  } 
  else {
    digitalWrite(RelayPin, LOW);
    // Serial.println("RELAY DECISION: --- LOW ---");
  }
  
  if (millis() > lastTime + 200){ //maybe change it so that it redraws those values every 200ms but data is actually logged every loop. This would make reaction to error faster.
    readSHT30Sensor();
    readNTCSensor();
    // unsigned int rpms = fan.getSpeed(); // Send the command to get RPM
    // byte dutyCycle = fan.getDutyCycle();
    // Serial.printf("Current speed: %5d RPM | Duty cycle: %3d%%\n", rpms, dutyCycle);
    lastTime = millis();
  }
  
  // byte target = max(min((int)targetFanSpeed, 100), 0);
  fan.setDutyCycle(targetFanSpeed);
    // Print obtained value
    // Serial.printf("Setting duty cycle: % 3d%%\n", target);
  // Serial.println("Heater Thermistor Temperature: "); Serial.print(heaterTempNTC); Serial.print("\t\t");
  // Serial.println("DRAWING");
  

  // if (buttonWasPressedThisLoop) {
  //     if (editingMode) {
  //     }
  //   }
  
  u8g2.firstPage();
  do {
      for (int i = 0; i < NUM_MENU_ITEMS; ++i) {
          int lineStartY = i * LINE_HEIGHT;
          char prefix = (editingMode && i == editingLine) ? '>' : ' ';

          switch (i) {
              case 0:
                  // snprintf(displayBuffer, sizeof(displayBuffer), "%c%s %.1fC", prefix, itemLabels[i], targetTemperature);
                  sprintf(displayBuffer, "Current Temp: %.1f C", enclosureTempSHT30);
                  break;
              case 1:
                  // snprintf(displayBuffer, sizeof(displayBuffer), "%c%s %d%%", prefix, itemLabels[i], targetFanSpeed);
                  //Heater on off
                  if (heaterEnabled){
                    sprintf(displayBuffer, "Heater ON");
                  }

                  else{
                    sprintf(displayBuffer, "Heater OFF");
                  }

                  break;
              case 2:
                  // if (enclosureTempSHT30 <= -99.0) snprintf(displayBuffer, sizeof(displayBuffer), "%c%s --.-C", prefix, itemLabels[i]);
                  // else snprintf(displayBuffer, sizeof(displayBuffer), "%c%s %.1fC", prefix, itemLabels[i], enclosureTempSHT30);
                  sprintf(displayBuffer, "Target Temp: %.1f C", targetTemperature);
                  break;
              case 3:
                  
                sprintf(displayBuffer, "Fan Speed:  %d %%", targetFanSpeed); // Added space for alignment
              //       if (heaterTempNTC <= -99.0) snprintf(displayBuffer, sizeof(displayBuffer), "%c%s --.-C", prefix, itemLabels[i]);
              //     else snprintf(displayBuffer, sizeof(displayBuffer), "%c%s %.1fC", prefix, itemLabels[i], heaterTempNTC);
                  break;
              case 4:
                  // if (enclosureHumiditySHT30 <= -99.0) snprintf(displayBuffer, sizeof(displayBuffer), "%c%s --.-%%", prefix, itemLabels[i]);
                  // else snprintf(displayBuffer, sizeof(displayBuffer), "%c%s %.0f%%", prefix, itemLabels[i], enclosureHumiditySHT30);
                  sprintf(displayBuffer, "Humidity: %.2f %%", enclosureHumiditySHT30);
                  break;

              case 5:
                sprintf(displayBuffer, "Heater Core Temp:  %.1f C", heaterTempNTC);
                break;
          }

          if (i == selectedLine) {
              u8g2.setDrawColor(1);
              u8g2.drawBox(0, lineStartY, DISPLAY_WIDTH, LINE_HEIGHT);
              u8g2.setDrawColor(0);
              u8g2.setCursor(TEXT_X_OFFSET, text_Y_baselines[i]);
              u8g2.print(displayBuffer);
              u8g2.setDrawColor(1);
          } else {
              u8g2.setDrawColor(1);
              u8g2.setCursor(TEXT_X_OFFSET, text_Y_baselines[i]);
              u8g2.print(displayBuffer);
          }
          yield(); 
      }
  } 
  while (u8g2.nextPage());


  
  
  



  }
