#include <Arduino.h>
#include <SPI.h>
#include <Wire.h> // For I2C
#include <U8g2lib.h>
#include <ESP32RotaryEncoder.h> // Using MaffooClock's ESP32RotaryEncoder library

#include <Adafruit_SHT31.h>
#include <NTC_Thermistor.h> // For the NTC thermistor

// --- Pin Definitions ---
// Display
#define DISPLAY_CS_PIN 5
#define DISPLAY_RST_PIN U8X8_PIN_NONE // For U8g2, U8X8_PIN_NONE if not used

// Encoder
const int ENCODER_A_PIN  = 25; // CLK for ESP32RotaryEncoder
const int ENCODER_B_PIN  = 26; // DT for ESP32RotaryEncoder
const int ENCODER_SW_PIN = 27; // Switch / Button
const int ENCODER_STEPS_PER_DETENT = 4; // Pulses per physical click/detent

// NTC Thermistor Pin (ensure this is an ADC capable pin on your ESP32, e.g., GPIO34)
const int NTC_SENSOR_PIN = 34;

// --- NTC Thermistor Configuration ---
#define NTC_REFERENCE_RESISTANCE    4700  // Value of the series resistor in Ohms (e.g., 4.7kOhms or 10kOhms)
#define NTC_NOMINAL_RESISTANCE      100000 // Nominal resistance of the thermistor at nominal temperature (e.g., 100kOhms for a 3950 NTC)
#define NTC_NOMINAL_TEMPERATURE     25    // Nominal temperature for the thermistor in Celsius (e.g., 25 C)
#define NTC_B_VALUE                 3950  // Beta coefficient (B-value) of the thermistor
#define NTC_ESP32_ANALOG_RESOLUTION 4095  // ADC resolution for ESP32 (12-bit ADC, 0-4095)
#define NTC_ESP32_ADC_VREF_MV       3300  // ADC reference voltage in millivolts (e.g., 3300mV for 3.3V)

// --- Sensor Objects ---
Adafruit_SHT31 sht31 = Adafruit_SHT31();
Thermistor* ntc_thermistor; // Pointer for the NTC thermistor object

// --- U8g2 Display Object ---
U8G2_ST7920_128X64_F_HW_SPI u8g2(U8G2_R0, DISPLAY_CS_PIN, DISPLAY_RST_PIN);

// --- Menu and Display Layout ---
const int NUM_MENU_ITEMS = 5;
const int LINE_HEIGHT = 12;
const int DISPLAY_WIDTH = 128;
const int TEXT_X_OFFSET = 2;
int text_Y_baselines[NUM_MENU_ITEMS];

const char* itemLabels[NUM_MENU_ITEMS] = {
  "Set Temp:", "Set Fan:", "Env Temp:", "NTC Temp:", "Env Humi:"
};
bool itemEditable[NUM_MENU_ITEMS] = {
  true, true, false, false, false
};

// --- Data Variables ---
float targetTemperature = 60.0;
int   targetFanSpeed = 50;
float enclosureTempSHT30 = -99.9;
float enclosureHumiditySHT30 = -99.9;
float heaterTempNTC = -99.9;

char displayBuffer[48]; // Buffer for formatting strings for display (ensure it's large enough)

// --- Encoder and Editing State ---
int selectedLine = 0;
bool editingMode = false;
int editingLine = -1;
long encoderValueAtEditStart = 0;
float initialValueForEditingFloat = 0.0;
int initialValueForEditingInt = 0;

volatile bool buttonPressedFlag = false;
volatile bool encoderTurnedFlag = false; // Optional

// --- RotaryEncoder Object ---
RotaryEncoder rotaryEncoder(ENCODER_A_PIN, ENCODER_B_PIN, ENCODER_SW_PIN, ENCODER_STEPS_PER_DETENT);

// --- Sensor Reading Timing ---
unsigned long lastShtReadTime = 0;
unsigned long lastNtcReadTime = 0;
// Increased interval for debugging to reduce system load
const unsigned long SENSOR_READ_INTERVAL_MS = 2000; // Read sensors every 2 seconds

/**
 * @brief Callback function for RotaryEncoder rotation (onTurned).
 * @param value The new encoder value (after steps division).
 * ISRs should be in IRAM.
 */
void IRAM_ATTR rotary_onTurned_callback(long value) {
    encoderTurnedFlag = true;
}

/**
 * @brief Callback function for the encoder button press (onPressed).
 * @param duration How long the button was held down (in milliseconds).
 * ISRs should be in IRAM.
 */
void IRAM_ATTR rotary_onPressed_callback(unsigned long duration) {
    buttonPressedFlag = true;
}

// Function to read SHT30 sensor data
void readSHT30Sensor() {
    enclosureTempSHT30 = sht31.readTemperature();
    enclosureHumiditySHT30 = sht31.readHumidity();
    if (isnan(enclosureTempSHT30)) {
        enclosureTempSHT30 = -99.9;
        Serial.println("Failed to read temperature from SHT30");
    }
    if (isnan(enclosureHumiditySHT30)) {
        enclosureHumiditySHT30 = -99.9;
        Serial.println("Failed to read humidity from SHT30");
    }
}

// Function to read NTC sensor data
void readNTCSensor() {
    if (ntc_thermistor != nullptr) { // Check if the object was created
        heaterTempNTC = ntc_thermistor->readCelsius();
        if (isnan(heaterTempNTC)) {
            heaterTempNTC = -99.9;
            Serial.println("Failed to read temperature from NTC");
        }
    } else {
        heaterTempNTC = -99.9;
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println("ESP32 Menu & Sensor Control - NTC Integrated (Spinlock Fix Attempt)");

    Wire.begin();
    if (!sht31.begin(0x44)) {
        Serial.println("Couldn't find SHT31 sensor at 0x44!");
    } else {
        Serial.println("SHT31 sensor found at 0x44!");
    }

    Serial.println("Initializing NTC Thermistor...");
    ntc_thermistor = new NTC_Thermistor_ESP32(
        NTC_SENSOR_PIN,
        (double)NTC_REFERENCE_RESISTANCE,
        (double)NTC_NOMINAL_RESISTANCE,
        (double)NTC_NOMINAL_TEMPERATURE,
        (double)NTC_B_VALUE,
        NTC_ESP32_ADC_VREF_MV,
        NTC_ESP32_ANALOG_RESOLUTION
    );
    Serial.println("NTC Thermistor initialized.");
    
    rotaryEncoder.setEncoderType(EncoderType::HAS_PULLUP);
    rotaryEncoder.onTurned(rotary_onTurned_callback);
    rotaryEncoder.onPressed(rotary_onPressed_callback);
    rotaryEncoder.setBoundaries(0, NUM_MENU_ITEMS - 1, true);
    rotaryEncoder.setEncoderValue(0);
    rotaryEncoder.begin();
    
    selectedLine = 0;

    SPI.begin();
    u8g2.begin();
    u8g2.setBusClock(1000000); 
    u8g2.setFont(u8g2_font_helvR10_tf);

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
    
    Serial.println("Setup complete.");

    readSHT30Sensor();
    readNTCSensor();
    lastShtReadTime = millis();
    lastNtcReadTime = millis();
}

long lastKnownEncoderValue = 0; 

void loop() {
    bool buttonWasPressedThisLoop = false;
    // Removed noInterrupts()/interrupts() block.
    // Rely on volatile and simple assignment for flag handling.
    if (buttonPressedFlag) {
        buttonWasPressedThisLoop = true;
        buttonPressedFlag = false; // Clear the flag
    }

    long currentEncoderValue = rotaryEncoder.getEncoderValue();

    if (buttonWasPressedThisLoop) {
        if (editingMode) {
            editingMode = false;
            editingLine = -1;
            rotaryEncoder.setBoundaries(0, NUM_MENU_ITEMS - 1, true);
            rotaryEncoder.setEncoderValue(selectedLine);
            lastKnownEncoderValue = rotaryEncoder.getEncoderValue(); 
        } else {
            if (itemEditable[selectedLine]) {
                editingMode = true;
                editingLine = selectedLine;
                encoderValueAtEditStart = currentEncoderValue; 
                if (editingLine == 0) {
                    initialValueForEditingFloat = targetTemperature;
                } else if (editingLine == 1) {
                    initialValueForEditingInt = targetFanSpeed;
                }
                rotaryEncoder.setBoundaries(-10000, 10000, false);
            }
        }
    }

    if (currentEncoderValue != lastKnownEncoderValue) {
        if (editingMode) {
            long detent_delta = currentEncoderValue - encoderValueAtEditStart;
            if (editingLine == 0) {
                targetTemperature = initialValueForEditingFloat + (float)detent_delta * 0.5;
                if (targetTemperature < 0.0) targetTemperature = 0.0;
                if (targetTemperature > 250.0) targetTemperature = 250.0;
            } else if (editingLine == 1) {
                targetFanSpeed = initialValueForEditingInt + (int)detent_delta * 5;
                if (targetFanSpeed < 0) targetFanSpeed = 0;
                if (targetFanSpeed > 100) targetFanSpeed = 100;
            }
        } else {
            selectedLine = currentEncoderValue;
        }
        lastKnownEncoderValue = currentEncoderValue;
        encoderTurnedFlag = false; 
    }

    unsigned long currentTime = millis();
    if (currentTime - lastShtReadTime >= SENSOR_READ_INTERVAL_MS) {
        readSHT30Sensor();
        lastShtReadTime = currentTime;
    }
    if (currentTime - lastNtcReadTime >= SENSOR_READ_INTERVAL_MS) {
        readNTCSensor();
        lastNtcReadTime = currentTime;
    }

    u8g2.firstPage();
    do {
        for (int i = 0; i < NUM_MENU_ITEMS; ++i) {
            int lineStartY = i * LINE_HEIGHT;
            char prefix = (editingMode && i == editingLine) ? '>' : ' ';

            switch (i) {
                case 0:
                    snprintf(displayBuffer, sizeof(displayBuffer), "%c%s %.1fC", prefix, itemLabels[i], targetTemperature);
                    break;
                case 1:
                    snprintf(displayBuffer, sizeof(displayBuffer), "%c%s %d%%", prefix, itemLabels[i], targetFanSpeed);
                    break;
                case 2:
                    if (enclosureTempSHT30 <= -99.0) snprintf(displayBuffer, sizeof(displayBuffer), "%c%s --.-C", prefix, itemLabels[i]);
                    else snprintf(displayBuffer, sizeof(displayBuffer), "%c%s %.1fC", prefix, itemLabels[i], enclosureTempSHT30);
                    break;
                case 3:
                     if (heaterTempNTC <= -99.0) snprintf(displayBuffer, sizeof(displayBuffer), "%c%s --.-C", prefix, itemLabels[i]);
                    else snprintf(displayBuffer, sizeof(displayBuffer), "%c%s %.1fC", prefix, itemLabels[i], heaterTempNTC);
                    break;
                case 4:
                    if (enclosureHumiditySHT30 <= -99.0) snprintf(displayBuffer, sizeof(displayBuffer), "%c%s --.-%%", prefix, itemLabels[i]);
                    else snprintf(displayBuffer, sizeof(displayBuffer), "%c%s %.0f%%", prefix, itemLabels[i], enclosureHumiditySHT30);
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
    } while (u8g2.nextPage());

    delay(20);
}
