#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_INA228.h>
#include <EEPROM.h>
#include <avr/wdt.h>
#include <math.h>

//==============================================================================
// --- CONFIGURATION CONSTANTS ---
//==============================================================================

// Pin Definitions
static const struct Pins {
  const uint8_t MOSFET = 6;
  const uint8_t INDICATOR_LED = 12;
  const uint8_t TOGGLE_BUTTON = 25;
  const uint8_t NEXT_VOLTAGE = 17;
  const uint8_t PREV_VOLTAGE = 16;
} PINS;

// I2C Address Definitions
static const struct I2C {
  const uint8_t OLED = 0x3C;
  const uint8_t HUSB238A = 0x62;
  const uint8_t INA228 = 0x40;
} I2C_ADDR;

// General & Sensor Configuration
static const struct Config {
  const uint16_t PRESS_THRESHOLD = 1020;
  const float SHUNT_RESISTOR_OHMS = 0.00702f;
  const uint16_t SHUNT_TEMPCO_PPM_C = 10; // Tempco of shunt in ppm/°C
  const float MAX_SAFE_VOLTAGE = 48.0f;
  const float MAX_SAFE_TEMPERATURE_C = 85.0f;
  const uint32_t EEPROM_MAGIC_NUMBER = 0xABCDEED2; // Changed to invalidate old EEPROM
  const unsigned long DEBOUNCE_DELAY_MS = 10;
  const unsigned long LONG_PRESS_MS = 400;
  const unsigned long DISPLAY_UPDATE_INTERVAL_MS = 150;
} CONFIG;

// PPS Mode Configuration
static const struct PPSConfig {
  const float MIN_VOLTAGE = 4.0f;
  const float MAX_VOLTAGE = 28.0f;
  const float STEP_VOLTAGE = 0.1f;
  const uint16_t REQUEST_CURRENT_MA = 3000;
  const unsigned long FAST_STEP_DELAY_MS = 500;
  const unsigned long FAST_STEP_INTERVAL_MS = 2;
  const uint8_t FAST_STEP_MULTIPLIER = 1;
} PPS_CONFIG;

// AVS Mode Configuration
static const struct AVSConfig {
  const float STEP_VOLTAGE = 0.1f;
  const uint16_t REQUEST_CURRENT_MA = 3000;
  const unsigned long FAST_STEP_INTERVAL_MS = 2;
  const uint8_t FAST_STEP_MULTIPLIER = 1;
} AVS_CONFIG;

// PSU Mode Configuration
static const struct PSUConfig {
  const float MIN_VOLTAGE = 4.0f;
  const float MAX_VOLTAGE = 28.0f;
  const float VOLTAGE_STEP = 0.10f;
  const uint16_t CURRENT_STEP_MA = 50;
  const uint16_t MAX_CURRENT_MA = 5000;

  // PID Profile for startup (fast, no-load)
  const float KP_VOLTAGE_COMP_STARTUP = 0.0f;
  const float KI_VOLTAGE_COMP_STARTUP = 0.5f;
  const float KD_VOLTAGE_COMP_STARTUP = 0.0f;

  // PID Profile for load response (to fight Vdroop)
  const float KP_VOLTAGE_COMP_LOAD = 0.2f;
  const float KI_VOLTAGE_COMP_LOAD = 5.0f;
  const float KD_VOLTAGE_COMP_LOAD = 0.0f;

  // Threshold to detect a new load (in mA)
  const float LOAD_CHANGE_THRESHOLD_MA = 100.0f;

  const float PSU_VOLTAGE_INTEGRAL_LIMIT = 4.0f;
  const unsigned long PSU_UPDATE_INTERVAL_MS = 20;
  const unsigned long PSU_CC_UPDATE_INTERVAL_MS = 10;
  const float KP_CURRENT = 0.0015f;
  const float KI_CURRENT = 0.000f;
  const float KD_CURRENT = 0.003f;
  const unsigned long FAST_STEP_INTERVAL_MS = 2;
  const uint8_t FAST_STEP_MULTIPLIER = 1;
} PSU_CONFIG;


// Display Layout
static const struct DisplayLayout {
  const uint8_t SCREEN_WIDTH = 128;
  const uint8_t SCREEN_HEIGHT = 64;
  const uint8_t LABEL_X = 0;
  const uint8_t VALUE_X = 36;
  const uint8_t VALUE2_X = 80;
  const uint8_t BAR_X = 34;
  const uint8_t BAR_Y = 50;
  const uint8_t BAR_WIDTH = 92;
  const uint8_t BAR_HEIGHT = 12;
} LAYOUT;

// HUSB238 Register Definitions
namespace HUSB238A {
  const uint8_t REG_CONTROL1 = 0x02;
  const uint8_t REG_GO_COMMAND = 0x18;
  const uint8_t REG_SRC_PDO = 0x19;
  const uint8_t REG_USER_CFG3 = 0x0F;
  const uint8_t REG_SNK_PPS_VOLTAGE = 0x1A;
  const uint8_t REG_SNK_PPS_CURRENT = 0x1B;
  const uint8_t REG_SNK_AVS_VOLTAGE = 0x1C;
  const uint8_t REG_SNK_AVS_CURRENT = 0x1D;
  const uint8_t REG_SRC_PDO_5V = 0x6A;
  const uint8_t REG_SRC_PDO_9V = 0x6B;
  const uint8_t REG_SRC_PDO_12V = 0x6C;
  const uint8_t REG_SRC_PDO_15V = 0x6D;
  const uint8_t REG_SRC_PDO_20V = 0x6E;
  const uint8_t REG_SRC_PDO_28V = 0x6F;
  const uint8_t REG_SRC_PDO_36V = 0x70;
  const uint8_t REG_SRC_PDO_48V = 0x71;
  const uint8_t REG_SRC_PDO_PPS1_STATUS = 0x72;
  const uint8_t REG_SRC_PDO_PPS2_STATUS = 0x73;
  const uint8_t REG_SRC_PDO_PPS3_STATUS = 0x74;
  const uint8_t REG_SRC_PPS_VOLTAGE_CAP = 0x75;
  const uint8_t REG_SRC_PDO_AVS_STATUS = 0x76;
  const uint8_t REG_SRC_AVS_PDP = 0x77;
}

// EEPROM Address Definitions
namespace EEP {
  const int ADDR_MAGIC = 0;
  const int ADDR_MODE = 4;
  const int ADDR_FIXED_IDX = 5;
  const int ADDR_PPS_VOLTAGE = 6;
  const int ADDR_MOSFET_STATE = 11;
  const int ADDR_PSU_TARGET_V = 12;
  const int ADDR_PSU_TARGET_I = 16;
  const int ADDR_AVS_VOLTAGE = 20;
}

//==============================================================================
// --- ENUMS & GLOBAL VARIABLES ---
//==============================================================================

enum ErrorCondition { NO_ERROR, HALT_HUSB_FAULT, HALT_INA_FAULT, HALT_OLED_FAULT, HALT_OVER_TEMP, HALT_OVER_VOLTAGE, HALT_SENSOR_FAILURE };

Adafruit_SSD1306 display(LAYOUT.SCREEN_WIDTH, LAYOUT.SCREEN_HEIGHT, &Wire, -1);
Adafruit_INA228 ina228 = Adafruit_INA228();
struct Button { const uint8_t PIN; bool debouncedState; bool lastReading; unsigned long lastDebounceTime; unsigned long pressStartTime; bool isPressing; bool longPressHandled; };
Button toggleButton = {PINS.TOGGLE_BUTTON, false, false, 0, 0, false, false};
Button nextButton = {PINS.NEXT_VOLTAGE, false, false, 0, 0, false, false};
Button prevButton = {PINS.PREV_VOLTAGE, false, false, 0, 0, false, false};

enum OperatingMode { MODE_FIXED_PD, MODE_PPS, MODE_PSU, MODE_AVS };
enum PSUAdjustMode { ADJUST_V, ADJUST_I };
struct VoltageProfile { const char* name; float voltage; uint8_t pdo_select_val; uint8_t status_reg_addr; float max_current; };
struct PPSProfile { float minVoltage; float maxVoltage; float maxCurrent; bool supported; };
struct AVSProfile { bool supported; float minVoltage; float maxVoltage; float maxCurrent; };
struct SensorReadings { float voltage; float current; float power; float temperature; };

VoltageProfile profiles_to_check[] = {
  {"5V",  5.0f,  (0b00001 << 3), HUSB238A::REG_SRC_PDO_5V,  0.0f},
  {"9V",  9.0f,  (0b00010 << 3), HUSB238A::REG_SRC_PDO_9V,  0.0f},
  {"12V", 12.0f, (0b00011 << 3), HUSB238A::REG_SRC_PDO_12V, 0.0f},
  {"15V", 15.0f, (0b00100 << 3), HUSB238A::REG_SRC_PDO_15V, 0.0f},
  {"20V", 20.0f, (0b00101 << 3), HUSB238A::REG_SRC_PDO_20V, 0.0f},
  {"28V", 28.0f, (0b11000 << 3), HUSB238A::REG_SRC_PDO_28V, 0.0f},
  {"36V", 36.0f, (0b11010 << 3), HUSB238A::REG_SRC_PDO_36V, 0.0f},
  {"48V", 48.0f, (0b11100 << 3), HUSB238A::REG_SRC_PDO_48V, 0.0f},
};
const int num_profiles_to_check = sizeof(profiles_to_check) / sizeof(VoltageProfile);
VoltageProfile supported_profiles[num_profiles_to_check];
int num_supported_profiles = 0;
PPSProfile pps_capability = {0.0f, 0.0f, 0.0f, false};
AVSProfile avs_capability = {false, 0.0f, 0.0f, 0.0f};
OperatingMode current_mode = MODE_FIXED_PD;
int current_profile_index = 0;
float current_requested_pps_voltage = 0.0f;
float current_requested_avs_voltage = 0.0f;
bool mosfetState = false;
PSUAdjustMode psu_adjust_mode = ADJUST_V;
float psu_target_voltage = 5.0f;
uint16_t psu_target_current_ma = 1000;
float psu_last_requested_voltage = 5.0f;
bool psu_in_cc_mode = false;
bool psu_cc_is_limited = false;
bool psu_cv_is_limited = false;
float psu_last_cc_error = 0.0f;
float psu_integral_error = 0.0f;
float psu_cc_voltage_request = 5.0f;
float psu_voltage_integral_error = 0.0f;
float psu_voltage_last_error = 0.0f;
unsigned long last_psu_voltage_update_time = 0;
unsigned long lastDisplayUpdateTime = 0;
unsigned long lastContinuousStepTime = 0;
unsigned long lastPsuUpdateTime = 0;
SensorReadings currentReadings;
bool settings_changed = false;
bool no_pd_source_flag = false;

// Variables to manage dual PID profiles for voltage compensation
enum VoltagePIDProfile { PID_STARTUP, PID_LOAD_RESPONSE };
VoltagePIDProfile current_pid_profile = PID_STARTUP;
float last_measured_current_ma = 0.0f;


//==============================================================================
// --- FUNCTION PROTOTYPES ---
//==============================================================================
void initHardware();
void configureINA228();
void detectPDProfiles();
void detectPPSProfile();
void detectAVSProfile();
void restoreState();
void applyInitialState();
void writeI2C8(uint8_t, uint8_t, uint8_t);
uint8_t readI2C8(uint8_t, uint8_t);
void requestFixedVoltage(uint8_t);
void requestPPSVoltage(float, uint16_t);
void requestAVSVoltage(float, uint16_t);
void handlePSUMode();
void applyPSUState();
void handleButton(Button*, void (*)(), void (*)());
void onToggleShortPress();
void onToggleLongPress();
void onNextPressAction();
void onPrevPressAction();
void onNextFastPressAction();
void onPrevFastPressAction();
void handleContinuousPress();
void saveStateToEEPROM();
void updateDisplay();
void halt_system(ErrorCondition);
void readAllSensors();
void recoverI2C();
void checkFailsafes();


//==============================================================================
// --- SETUP & MAIN LOOP ---
//==============================================================================
void setup() {
  wdt_disable();
  initHardware();
  detectPDProfiles();
  detectPPSProfile();
  detectAVSProfile();
  if (num_supported_profiles == 0 && !pps_capability.supported && !avs_capability.supported) {
    no_pd_source_flag = true;
  }

  mosfetState = false;
  restoreState();
  mosfetState = false; // Always start with FET off for safety

  applyInitialState();
  wdt_enable(WDTO_2S);
}

void loop() {
  wdt_reset();
  readAllSensors();

  if (no_pd_source_flag) {
    if (millis() - lastDisplayUpdateTime > CONFIG.DISPLAY_UPDATE_INTERVAL_MS) {
      lastDisplayUpdateTime = millis();
      updateDisplay();
    }
    return;
  }

  bool prev_next_button_state = nextButton.debouncedState;
  bool prev_prev_button_state = prevButton.debouncedState;
  handleButton(&toggleButton, onToggleShortPress, onToggleLongPress);
  handleButton(&nextButton, onNextPressAction, onNextFastPressAction);
  handleButton(&prevButton, onPrevPressAction, onPrevFastPressAction);
  handleContinuousPress();

  if (settings_changed) {
    if ((!nextButton.debouncedState && prev_next_button_state) || (!prevButton.debouncedState && prev_prev_button_state)) {
      saveStateToEEPROM();
      settings_changed = false;
    }
  }

  unsigned long currentTime = millis();
  static bool combo_pressed_last_cycle = false;
  bool combo_currently_pressed = nextButton.debouncedState && prevButton.debouncedState;
  if (current_mode == MODE_PSU) {
    if (combo_currently_pressed && !combo_pressed_last_cycle) {
      psu_adjust_mode = (psu_adjust_mode == ADJUST_V) ? ADJUST_I : ADJUST_V;
      updateDisplay();
    }
  }
  combo_pressed_last_cycle = combo_currently_pressed;

  if (current_mode == MODE_PSU && mosfetState) {
    if (currentTime - lastPsuUpdateTime > PSU_CONFIG.PSU_UPDATE_INTERVAL_MS) {
      lastPsuUpdateTime = currentTime;
      handlePSUMode();
    }
  }

  if (currentTime - lastDisplayUpdateTime > CONFIG.DISPLAY_UPDATE_INTERVAL_MS) {
    lastDisplayUpdateTime = currentTime;
    checkFailsafes();
    updateDisplay();
  }
}

//==============================================================================
// --- INITIALIZATION & HARDWARE ---
//==============================================================================
void initHardware() {
  pinMode(PINS.MOSFET, OUTPUT);
  digitalWrite(PINS.MOSFET, LOW);
  pinMode(PINS.INDICATOR_LED, OUTPUT);
  digitalWrite(PINS.INDICATOR_LED, LOW);
  pinMode(PINS.TOGGLE_BUTTON, INPUT);
  pinMode(PINS.NEXT_VOLTAGE, INPUT);
  pinMode(PINS.PREV_VOLTAGE, INPUT);
  Wire.begin();
  Wire.setClock(400000L);
  if (!display.begin(SSD1306_SWITCHCAPVCC, I2C_ADDR.OLED)) {
    while(1);
  }
  display.ssd1306_command(SSD1306_DISPLAYON);
  display.clearDisplay();
  display.display();
  if (!ina228.begin(I2C_ADDR.INA228)) {
    halt_system(HALT_INA_FAULT);
  }
  configureINA228();
  Wire.beginTransmission(I2C_ADDR.HUSB238A);
  if (Wire.endTransmission() != 0) {
    // This condition is handled gracefully after profile detection
  }
}

void configureINA228() {
  ina228.setADCRange(1);
  ina228.setAveragingCount(INA228_COUNT_4);
  ina228.setShunt(CONFIG.SHUNT_RESISTOR_OHMS);
  ina228.setMode(INA228_MODE_CONT_BUS_SHUNT);
  ina228.setVoltageConversionTime(INA228_TIME_1052_us);
  ina228.setCurrentConversionTime(INA228_TIME_1052_us);
}

void detectPDProfiles() {
  writeI2C8(I2C_ADDR.HUSB238A, HUSB238A::REG_USER_CFG3, (1 << 6) | (1 << 5));
  writeI2C8(I2C_ADDR.HUSB238A, HUSB238A::REG_CONTROL1, 0x0E);
  delay(100);
  writeI2C8(I2C_ADDR.HUSB238A, HUSB238A::REG_GO_COMMAND, 0x04);
  delay(500);
  num_supported_profiles = 0;
  for (int i = 0; i < num_profiles_to_check; i++) {
    uint8_t status = readI2C8(I2C_ADDR.HUSB238A, profiles_to_check[i].status_reg_addr);
    if ((status & 0x80) && (status != 0xFF) && (profiles_to_check[i].voltage <= CONFIG.MAX_SAFE_VOLTAGE)) {
      uint8_t current_val = status & 0x7F;
      profiles_to_check[i].max_current = (float)current_val * 0.1f;
      supported_profiles[num_supported_profiles++] = profiles_to_check[i];
    }
  }
}

void detectPPSProfile() {
  uint8_t pps1 = readI2C8(I2C_ADDR.HUSB238A, HUSB238A::REG_SRC_PDO_PPS1_STATUS);
  uint8_t pps2 = readI2C8(I2C_ADDR.HUSB238A, HUSB238A::REG_SRC_PDO_PPS2_STATUS);
  uint8_t pps3 = readI2C8(I2C_ADDR.HUSB238A, HUSB238A::REG_SRC_PDO_PPS3_STATUS);
  pps_capability.supported = ((pps1 | pps2 | pps3) & 0x80);
  if (!pps_capability.supported) return;
  uint8_t pps_v_cap = readI2C8(I2C_ADDR.HUSB238A, HUSB238A::REG_SRC_PPS_VOLTAGE_CAP);
  uint8_t min_v_code = pps_v_cap & 0x03;
  if (min_v_code == 0b00) pps_capability.minVoltage = 3.0f;
  else if (min_v_code == 0b01) pps_capability.minVoltage = 3.3f;
  else pps_capability.minVoltage = 5.0f;
  float detected_max_v = 0.0f;
  if (pps1 & 0x80) { uint8_t max_v = (pps_v_cap >> 6) & 0x03; if (max_v==3) detected_max_v=max(detected_max_v,21.f); else if (max_v==2) detected_max_v=max(detected_max_v,17.f); else if (max_v==1) detected_max_v=max(detected_max_v,12.f); else detected_max_v=max(detected_max_v,7.f); }
  if (pps2 & 0x80) { uint8_t max_v = (pps_v_cap >> 4) & 0x03; if (max_v==3) detected_max_v=max(detected_max_v,21.f); else if (max_v==2) detected_max_v=max(detected_max_v,17.f); else if (max_v==1) detected_max_v=max(detected_max_v,12.f); else detected_max_v=max(detected_max_v,7.f); }
  if (pps3 & 0x80) { uint8_t max_v = (pps_v_cap >> 2) & 0x03; if (max_v==3) detected_max_v=max(detected_max_v,21.f); else if (max_v==2) detected_max_v=max(detected_max_v,17.f); else if (max_v==1) detected_max_v=max(detected_max_v,12.f); else detected_max_v=max(detected_max_v,7.f); }
  pps_capability.maxVoltage = min(detected_max_v, CONFIG.MAX_SAFE_VOLTAGE);
  float max_pps_current = 0.0f;
  if (pps1 & 0x80) max_pps_current = max(max_pps_current, (float)(pps1 & 0x7F) * 0.1f);
  if (pps2 & 0x80) max_pps_current = max(max_pps_current, (float)(pps2 & 0x7F) * 0.1f);
  if (pps3 & 0x80) max_pps_current = max(max_pps_current, (float)(pps3 & 0x7F) * 0.1f);
  pps_capability.maxCurrent = max_pps_current;
}

void detectAVSProfile() {
  uint8_t avs_status = readI2C8(I2C_ADDR.HUSB238A, HUSB238A::REG_SRC_PDO_AVS_STATUS);
  avs_capability.supported = (avs_status & 0x80);
  if (!avs_capability.supported) return;

  uint8_t min_v_code = avs_status & 0x03;
  if (min_v_code == 0b00) avs_capability.minVoltage = 5.0f;
  else if (min_v_code == 0b01) avs_capability.minVoltage = 9.0f;
  else avs_capability.minVoltage = 15.0f;

  uint8_t max_v_code = (avs_status >> 3) & 0x0F;
  avs_capability.maxVoltage = 5.0f + (float)max_v_code;

  uint8_t pdp_val = readI2C8(I2C_ADDR.HUSB238A, HUSB238A::REG_SRC_AVS_PDP);
  if (pdp_val > 0 && avs_capability.minVoltage > 0) {
    avs_capability.maxCurrent = (float)pdp_val / avs_capability.minVoltage;
  } else {
    avs_capability.maxCurrent = 0;
  }
}


void restoreState() {
  if (no_pd_source_flag) {
    current_mode = MODE_FIXED_PD;
    current_profile_index = 0;
    return;
  }
  uint32_t magic;
  EEPROM.get(EEP::ADDR_MAGIC, magic);
  if (magic == CONFIG.EEPROM_MAGIC_NUMBER) {
    EEPROM.get(EEP::ADDR_MODE, (uint8_t&)current_mode);
    EEPROM.get(EEP::ADDR_FIXED_IDX, (uint8_t&)current_profile_index);
    EEPROM.get(EEP::ADDR_PPS_VOLTAGE, current_requested_pps_voltage);
    EEPROM.get(EEP::ADDR_AVS_VOLTAGE, current_requested_avs_voltage);
    EEPROM.get(EEP::ADDR_PSU_TARGET_V, psu_target_voltage);
    EEPROM.get(EEP::ADDR_PSU_TARGET_I, psu_target_current_ma);
  } else {
    current_mode = MODE_FIXED_PD;
    current_profile_index = 0;
    current_requested_pps_voltage = PPS_CONFIG.MIN_VOLTAGE;
    current_requested_avs_voltage = 5.0f;
    mosfetState = false;
    psu_target_voltage = 5.0f;
    psu_target_current_ma = 1000;
    saveStateToEEPROM();
    return;
  }
  bool state_was_corrected = false;
  if (current_mode == MODE_FIXED_PD) {
    if (num_supported_profiles > 0 && current_profile_index >= num_supported_profiles) {
      current_profile_index = 0;
      state_was_corrected = true;
    }
  } else if (current_mode == MODE_PPS || current_mode == MODE_PSU) {
    if (!pps_capability.supported) {
      current_mode = MODE_FIXED_PD;
      current_profile_index = 0;
      state_was_corrected = true;
    } else {
      float pps_min = max(pps_capability.minVoltage, PPS_CONFIG.MIN_VOLTAGE);
      if (current_requested_pps_voltage < pps_min || current_requested_pps_voltage > pps_capability.maxVoltage) {
        current_requested_pps_voltage = constrain(current_requested_pps_voltage, pps_min, pps_capability.maxVoltage);
        state_was_corrected = true;
      }
      if (psu_target_voltage < pps_min || psu_target_voltage > pps_capability.maxVoltage) {
        psu_target_voltage = constrain(psu_target_voltage, pps_min, pps_capability.maxVoltage);
        state_was_corrected = true;
      }
      if (psu_target_current_ma > (pps_capability.maxCurrent * 1000) || psu_target_current_ma > PSU_CONFIG.MAX_CURRENT_MA) {
        psu_target_current_ma = min((uint16_t)(pps_capability.maxCurrent * 1000), PSU_CONFIG.MAX_CURRENT_MA);
        state_was_corrected = true;
      }
    }
  } else if (current_mode == MODE_AVS) {
    if (!avs_capability.supported) {
      current_mode = MODE_FIXED_PD;
      current_profile_index = 0;
      state_was_corrected = true;
    } else {
      if (current_requested_avs_voltage < avs_capability.minVoltage || current_requested_avs_voltage > avs_capability.maxVoltage) {
        current_requested_avs_voltage = constrain(current_requested_avs_voltage, avs_capability.minVoltage, avs_capability.maxVoltage);
        state_was_corrected = true;
      }
    }
  }

  if (state_was_corrected) saveStateToEEPROM();
}

void applyInitialState() {
  if (no_pd_source_flag) {
    digitalWrite(PINS.MOSFET, LOW);
    digitalWrite(PINS.INDICATOR_LED, LOW);
    mosfetState = false;
    return;
  }
  digitalWrite(PINS.MOSFET, mosfetState);
  digitalWrite(PINS.INDICATOR_LED, mosfetState);
  if (mosfetState) {
    if (current_mode == MODE_FIXED_PD) {
      if (num_supported_profiles > 0) requestFixedVoltage(supported_profiles[current_profile_index].pdo_select_val);
    } else if (current_mode == MODE_PPS) {
      if (pps_capability.supported) requestPPSVoltage(current_requested_pps_voltage, PPS_CONFIG.REQUEST_CURRENT_MA);
    } else if (current_mode == MODE_PSU) {
      if (pps_capability.supported) applyPSUState();
    } else if (current_mode == MODE_AVS) {
      if (avs_capability.supported) requestAVSVoltage(current_requested_avs_voltage, AVS_CONFIG.REQUEST_CURRENT_MA);
    }
  } else {
    uint8_t five_volt_pdo_mask = (0b00001 << 3);
    for (int i = 0; i < num_supported_profiles; i++) {
      if (supported_profiles[i].voltage == 5.0f) {
        five_volt_pdo_mask = supported_profiles[i].pdo_select_val;
        break;
      }
    }
    requestFixedVoltage(five_volt_pdo_mask);
  }
}

void requestFixedVoltage(uint8_t pdo_select_mask) {
  uint8_t reg_val = readI2C8(I2C_ADDR.HUSB238A, HUSB238A::REG_SRC_PDO) & 0x07;
  reg_val |= pdo_select_mask;
  writeI2C8(I2C_ADDR.HUSB238A, HUSB238A::REG_SRC_PDO, reg_val);
  writeI2C8(I2C_ADDR.HUSB238A, HUSB238A::REG_GO_COMMAND, 0x01);
  delay(200);
}

void requestPPSVoltage(float targetVoltage, uint16_t targetCurrent_mA) {
  float pps_min = max(pps_capability.minVoltage, PPS_CONFIG.MIN_VOLTAGE);
  targetVoltage = constrain(targetVoltage, pps_min, pps_capability.maxVoltage);
  if (current_mode == MODE_PPS) current_requested_pps_voltage = targetVoltage;
  psu_last_requested_voltage = targetVoltage;
  uint16_t voltage_10bit = min((uint16_t)roundf((targetVoltage - 3.0f) / 0.02f), (uint16_t)0x3FF);
  uint8_t current_7bit = min((uint8_t)roundf(targetCurrent_mA / 50.0f), (uint8_t)0x7F);
  writeI2C8(I2C_ADDR.HUSB238A, HUSB238A::REG_SNK_PPS_VOLTAGE, voltage_10bit & 0xFF);
  writeI2C8(I2C_ADDR.HUSB238A, HUSB238A::REG_SNK_PPS_CURRENT, current_7bit);
  uint8_t reg_val = readI2C8(I2C_ADDR.HUSB238A, HUSB238A::REG_SRC_PDO) & 0x04;
  reg_val |= ((voltage_10bit >> 8) & 0x03);
  reg_val |= (0x06 << 3);
  writeI2C8(I2C_ADDR.HUSB238A, HUSB238A::REG_SRC_PDO, reg_val);
  writeI2C8(I2C_ADDR.HUSB238A, HUSB238A::REG_GO_COMMAND, 0x01);
}

void requestAVSVoltage(float targetVoltage, uint16_t targetCurrent_mA) {
  targetVoltage = constrain(targetVoltage, avs_capability.minVoltage, avs_capability.maxVoltage);
  if (current_mode == MODE_AVS) current_requested_avs_voltage = targetVoltage;

  uint8_t voltage_8bit = min((uint8_t)roundf(targetVoltage / 0.1f), (uint8_t)0xFF);
  uint8_t current_7bit = min((uint8_t)roundf(targetCurrent_mA / 50.0f), (uint8_t)0x7F);

  writeI2C8(I2C_ADDR.HUSB238A, HUSB238A::REG_SNK_AVS_VOLTAGE, voltage_8bit);
  writeI2C8(I2C_ADDR.HUSB238A, HUSB238A::REG_SNK_AVS_CURRENT, current_7bit);

  uint8_t reg_val = readI2C8(I2C_ADDR.HUSB238A, HUSB238A::REG_SRC_PDO) & 0x07;
  reg_val |= (0b01001 << 3); // PDO_SELECT value for AVS
  writeI2C8(I2C_ADDR.HUSB238A, HUSB238A::REG_SRC_PDO, reg_val);

  writeI2C8(I2C_ADDR.HUSB238A, HUSB238A::REG_GO_COMMAND, 0x01); // GO
}


void handlePSUMode() {
  if (!mosfetState || !pps_capability.supported || isnan(currentReadings.current)) {
    psu_in_cc_mode = false;
    psu_cc_is_limited = false;
    psu_cv_is_limited = false;
    psu_last_cc_error = 0.0f;
    psu_integral_error = 0.0f;
    psu_voltage_integral_error = 0.0f;
    return;
  }

  float measured_current_ma = fabs(currentReadings.current);

  // --- PID Profile Switching Logic ---
  if (current_pid_profile == PID_STARTUP && (measured_current_ma - last_measured_current_ma) > PSU_CONFIG.LOAD_CHANGE_THRESHOLD_MA) {
    current_pid_profile = PID_LOAD_RESPONSE;
    psu_voltage_integral_error = 0; // Reset integral to prevent jump
  } else if (current_pid_profile == PID_LOAD_RESPONSE && measured_current_ma < PSU_CONFIG.LOAD_CHANGE_THRESHOLD_MA) {
    current_pid_profile = PID_STARTUP;
  }
  last_measured_current_ma = measured_current_ma;


  if (psu_in_cc_mode) {
    psu_cv_is_limited = false;
    float current_error = measured_current_ma - psu_target_current_ma;

    if (psu_cc_voltage_request >= psu_target_voltage && current_error < -10) {
      psu_in_cc_mode = false;
      applyPSUState();
      return;
    }

    float pps_min_v = max(pps_capability.minVoltage, PSU_CONFIG.MIN_VOLTAGE);
    float error_derivative = current_error - psu_last_cc_error;
    float voltage_correction = (current_error * PSU_CONFIG.KP_CURRENT) + (error_derivative * PSU_CONFIG.KD_CURRENT);

    psu_cc_voltage_request -= voltage_correction;
    psu_last_cc_error = current_error;

    float original_request = psu_cc_voltage_request;

    psu_cc_voltage_request = constrain(psu_cc_voltage_request, pps_min_v, psu_target_voltage);

    psu_cc_is_limited = (psu_cc_voltage_request <= pps_min_v && original_request < pps_min_v);

    requestPPSVoltage(psu_cc_voltage_request, psu_target_current_ma);

  } else { // CV Mode
    if (measured_current_ma > (psu_target_current_ma + 25)) {
      psu_in_cc_mode = true;
      psu_cc_voltage_request = currentReadings.voltage;
      psu_last_cc_error = 0.0f;
      psu_integral_error = 0.0f;
      return;
    }

    unsigned long now = millis();
    float dt = (now - last_psu_voltage_update_time) / 1000.0f;
    last_psu_voltage_update_time = now;
    if (dt <= 0 || dt > 0.5f) {
      dt = PSU_CONFIG.PSU_UPDATE_INTERVAL_MS / 1000.0f;
    }

    float kp, ki, kd;
    if (current_pid_profile == PID_STARTUP) {
      kp = PSU_CONFIG.KP_VOLTAGE_COMP_STARTUP;
      ki = PSU_CONFIG.KI_VOLTAGE_COMP_STARTUP;
      kd = PSU_CONFIG.KD_VOLTAGE_COMP_STARTUP;
    } else { // PID_LOAD_RESPONSE
      kp = PSU_CONFIG.KP_VOLTAGE_COMP_LOAD;
      ki = PSU_CONFIG.KI_VOLTAGE_COMP_LOAD;
      kd = PSU_CONFIG.KD_VOLTAGE_COMP_LOAD;
    }

    float voltage_error = psu_target_voltage - currentReadings.voltage;
    psu_voltage_integral_error += voltage_error * dt;
    psu_voltage_integral_error = constrain(psu_voltage_integral_error, -PSU_CONFIG.PSU_VOLTAGE_INTEGRAL_LIMIT, PSU_CONFIG.PSU_VOLTAGE_INTEGRAL_LIMIT);
    float error_derivative = (voltage_error - psu_voltage_last_error) / dt;
    psu_voltage_last_error = voltage_error;

    float proportional_adj = voltage_error * kp;
    float integral_adj = psu_voltage_integral_error * ki;
    float derivative_adj = error_derivative * kd;

    float new_request_voltage = psu_target_voltage + proportional_adj + integral_adj + derivative_adj;

    if (new_request_voltage >= pps_capability.maxVoltage) {
      psu_cv_is_limited = true;
    } else {
      psu_cv_is_limited = false;
    }
    requestPPSVoltage(new_request_voltage, psu_target_current_ma);
  }
}

void applyPSUState() {
  if (!pps_capability.supported) return;
  psu_voltage_integral_error = 0.0f;
  psu_voltage_last_error = 0.0f;
  last_psu_voltage_update_time = millis();
  psu_in_cc_mode = false;
  psu_cc_is_limited = false;
  psu_cv_is_limited = false;
  psu_last_cc_error = 0.0f;
  psu_integral_error = 0.0f;

  // Reset to startup PID profile
  current_pid_profile = PID_STARTUP;
  last_measured_current_ma = 0.0f;

  requestPPSVoltage(psu_target_voltage, psu_target_current_ma);
  psu_cc_voltage_request = psu_target_voltage;
}


void handleButton(Button* btn, void (*onShortPress)(), void (*onLongPress)()) {
  bool currentReading = (analogRead(btn->PIN) > CONFIG.PRESS_THRESHOLD);
  if (currentReading != btn->lastReading) btn->lastDebounceTime = millis();
  if ((millis() - btn->lastDebounceTime) > CONFIG.DEBOUNCE_DELAY_MS) {
    if (currentReading != btn->debouncedState) {
      btn->debouncedState = currentReading;
      if (btn->debouncedState) {
        btn->isPressing = true;
        btn->pressStartTime = millis();
        btn->longPressHandled = false;
      } else {
        btn->isPressing = false;
        if (!btn->longPressHandled) onShortPress();
      }
    }
  }
  if (btn->isPressing && !btn->longPressHandled) {
    if ((millis() - btn->pressStartTime) > CONFIG.LONG_PRESS_MS) {
      onLongPress();
      btn->longPressHandled = true;
    }
  }
  btn->lastReading = currentReading;
}

void onToggleShortPress() {
  mosfetState = !mosfetState;
  digitalWrite(PINS.MOSFET, mosfetState);
  digitalWrite(PINS.INDICATOR_LED, mosfetState);
  if (mosfetState) {
    if (current_mode == MODE_PSU) {
      applyPSUState();
    } else if (current_mode == MODE_PPS) {
      requestPPSVoltage(current_requested_pps_voltage, PPS_CONFIG.REQUEST_CURRENT_MA);
    } else if (current_mode == MODE_AVS) {
      requestAVSVoltage(current_requested_avs_voltage, AVS_CONFIG.REQUEST_CURRENT_MA);
    } else {
      if (num_supported_profiles > 0) requestFixedVoltage(supported_profiles[current_profile_index].pdo_select_val);
    }
  } else {
    psu_voltage_integral_error = 0.0f;
    psu_voltage_last_error = 0.0f;
    psu_cv_is_limited = false;
    if (pps_capability.supported) {
      requestPPSVoltage(pps_capability.minVoltage, 500);
      delay(50);
    }
    uint8_t five_volt_pdo_mask = (0b00001 << 3);
    for (int i = 0; i < num_supported_profiles; i++) {
      if (supported_profiles[i].voltage == 5.0f) {
        five_volt_pdo_mask = supported_profiles[i].pdo_select_val;
        break;
      }
    }
    requestFixedVoltage(five_volt_pdo_mask);
  }
  saveStateToEEPROM();
  updateDisplay();
}

void onToggleLongPress() {
  // Always turn the output off when changing modes
  mosfetState = false;
  digitalWrite(PINS.MOSFET, false);
  digitalWrite(PINS.INDICATOR_LED, false);

  // First, cycle to the next valid mode
  int retries = 4;
  do {
    current_mode = (OperatingMode)(((int)current_mode + 1) % 4);
    retries--;
    if ( (current_mode == MODE_PPS || current_mode == MODE_PSU) && !pps_capability.supported) continue;
    if ( current_mode == MODE_AVS && !avs_capability.supported) continue;
    break;
  } while (retries > 0);

  // Reset PSU specific state when changing modes to ensure a clean start
  psu_adjust_mode = ADJUST_V;
  psu_voltage_integral_error = 0.0f;
  psu_voltage_last_error = 0.0f;
  psu_in_cc_mode = false;
  psu_cc_is_limited = false;
  psu_cv_is_limited = false;

  // Since the output is now off, negotiate the PD source back to a safe 5V level.
  // This mirrors the behavior of turning the power off with a short press.
  // The correct voltage for the new mode will be requested when the output is turned back on.
  if (pps_capability.supported) {
    // Some chargers need to be in a known state (like min PPS) before switching to fixed 5V
    requestPPSVoltage(pps_capability.minVoltage, 500);
    delay(50);
  }
  uint8_t five_volt_pdo_mask = (0b00001 << 3);
  for (int i = 0; i < num_supported_profiles; i++) {
    if (supported_profiles[i].voltage == 5.0f) {
      five_volt_pdo_mask = supported_profiles[i].pdo_select_val;
      break;
    }
  }
  requestFixedVoltage(five_volt_pdo_mask);

  saveStateToEEPROM();
  updateDisplay();
}


void onNextPressAction() {
  if (current_mode == MODE_PSU && psu_adjust_mode == ADJUST_V) {
    psu_target_voltage += PSU_CONFIG.VOLTAGE_STEP;
    psu_target_voltage = constrain(psu_target_voltage, PSU_CONFIG.MIN_VOLTAGE, pps_capability.maxVoltage);
    if (mosfetState && !psu_in_cc_mode) applyPSUState();
  } else if (current_mode == MODE_PSU && psu_adjust_mode == ADJUST_I) {
    psu_target_current_ma += PSU_CONFIG.CURRENT_STEP_MA;
    psu_target_current_ma = constrain(psu_target_current_ma, 0, PSU_CONFIG.MAX_CURRENT_MA);
  } else if (current_mode == MODE_FIXED_PD) {
    if (num_supported_profiles > 0) {
      current_profile_index = (current_profile_index + 1) % num_supported_profiles;
      if (mosfetState) requestFixedVoltage(supported_profiles[current_profile_index].pdo_select_val);
    }
  } else if (current_mode == MODE_PPS) {
    // *** MODIFIED: Check if max PPS voltage is reached and AVS is supported ***
    if (current_requested_pps_voltage >= pps_capability.maxVoltage && avs_capability.supported) {
        current_mode = MODE_AVS; // Switch to AVS mode
        current_requested_avs_voltage = avs_capability.minVoltage; // Start at AVS min
        if (mosfetState) requestAVSVoltage(current_requested_avs_voltage, AVS_CONFIG.REQUEST_CURRENT_MA);
    } else {
        current_requested_pps_voltage += PPS_CONFIG.STEP_VOLTAGE;
        current_requested_pps_voltage = constrain(current_requested_pps_voltage, pps_capability.minVoltage, pps_capability.maxVoltage);
        if (mosfetState) requestPPSVoltage(current_requested_pps_voltage, PPS_CONFIG.REQUEST_CURRENT_MA);
    }
  } else if (current_mode == MODE_AVS) {
    current_requested_avs_voltage += AVS_CONFIG.STEP_VOLTAGE;
    current_requested_avs_voltage = constrain(current_requested_avs_voltage, avs_capability.minVoltage, avs_capability.maxVoltage);
    if (mosfetState) requestAVSVoltage(current_requested_avs_voltage, AVS_CONFIG.REQUEST_CURRENT_MA);
  }
  updateDisplay();
  settings_changed = true;
}

void onPrevPressAction() {
  if (current_mode == MODE_PSU && psu_adjust_mode == ADJUST_V) {
    psu_target_voltage -= PSU_CONFIG.VOLTAGE_STEP;
    psu_target_voltage = constrain(psu_target_voltage, PSU_CONFIG.MIN_VOLTAGE, pps_capability.maxVoltage);
    if (mosfetState && !psu_in_cc_mode) applyPSUState();
  } else if (current_mode == MODE_PSU && psu_adjust_mode == ADJUST_I) {
    if (psu_target_current_ma >= PSU_CONFIG.CURRENT_STEP_MA) {
      psu_target_current_ma -= PSU_CONFIG.CURRENT_STEP_MA;
    } else {
      psu_target_current_ma = 0;
    }
  } else if (current_mode == MODE_FIXED_PD) {
    if (num_supported_profiles > 0) {
      current_profile_index = (current_profile_index > 0) ? current_profile_index - 1 : num_supported_profiles - 1;
      if (mosfetState) requestFixedVoltage(supported_profiles[current_profile_index].pdo_select_val);
    }
  } else if (current_mode == MODE_PPS){
    current_requested_pps_voltage -= PPS_CONFIG.STEP_VOLTAGE;
    current_requested_pps_voltage = constrain(current_requested_pps_voltage, pps_capability.minVoltage, pps_capability.maxVoltage);
    if (mosfetState) requestPPSVoltage(current_requested_pps_voltage, PPS_CONFIG.REQUEST_CURRENT_MA);
  } else if (current_mode == MODE_AVS) {
    // *** MODIFIED: Check if min AVS voltage is reached to return to PPS ***
    if (current_requested_avs_voltage <= avs_capability.minVoltage) {
        current_mode = MODE_PPS; // Switch back to PPS mode
        current_requested_pps_voltage = pps_capability.maxVoltage; // Go to max PPS voltage
        if (mosfetState) requestPPSVoltage(current_requested_pps_voltage, PPS_CONFIG.REQUEST_CURRENT_MA);
    } else {
        current_requested_avs_voltage -= AVS_CONFIG.STEP_VOLTAGE;
        current_requested_avs_voltage = constrain(current_requested_avs_voltage, avs_capability.minVoltage, avs_capability.maxVoltage);
        if (mosfetState) requestAVSVoltage(current_requested_avs_voltage, AVS_CONFIG.REQUEST_CURRENT_MA);
    }
  }
  updateDisplay();
  settings_changed = true;
}

void onNextFastPressAction() {
  if (current_mode == MODE_PSU && psu_adjust_mode == ADJUST_V) {
    // *** MODIFIED: Use new config parameter ***
    psu_target_voltage += (PSU_CONFIG.VOLTAGE_STEP * PSU_CONFIG.FAST_STEP_MULTIPLIER);
    psu_target_voltage = constrain(psu_target_voltage, PSU_CONFIG.MIN_VOLTAGE, pps_capability.maxVoltage);
    if (mosfetState && !psu_in_cc_mode) applyPSUState();
  } else if (current_mode == MODE_PSU && psu_adjust_mode == ADJUST_I) {
    // *** MODIFIED: Use new config parameter ***
    psu_target_current_ma += (PSU_CONFIG.CURRENT_STEP_MA * PSU_CONFIG.FAST_STEP_MULTIPLIER);
    psu_target_current_ma = constrain(psu_target_current_ma, 0, PSU_CONFIG.MAX_CURRENT_MA);
  } else if (current_mode == MODE_PPS) {
    current_requested_pps_voltage += (PPS_CONFIG.STEP_VOLTAGE * PPS_CONFIG.FAST_STEP_MULTIPLIER);
    current_requested_pps_voltage = constrain(current_requested_pps_voltage, pps_capability.minVoltage, pps_capability.maxVoltage);
    if (mosfetState) requestPPSVoltage(current_requested_pps_voltage, PPS_CONFIG.REQUEST_CURRENT_MA);
  } else if (current_mode == MODE_AVS) {
    current_requested_avs_voltage += (AVS_CONFIG.STEP_VOLTAGE * AVS_CONFIG.FAST_STEP_MULTIPLIER);
    current_requested_avs_voltage = constrain(current_requested_avs_voltage, avs_capability.minVoltage, avs_capability.maxVoltage);
    if (mosfetState) requestAVSVoltage(current_requested_avs_voltage, AVS_CONFIG.REQUEST_CURRENT_MA);
  }
  settings_changed = true;
}

void onPrevFastPressAction() {
  if (current_mode == MODE_PSU && psu_adjust_mode == ADJUST_V) {
    // *** MODIFIED: Use new config parameter ***
    psu_target_voltage -= (PSU_CONFIG.VOLTAGE_STEP * PSU_CONFIG.FAST_STEP_MULTIPLIER);
    psu_target_voltage = constrain(psu_target_voltage, PSU_CONFIG.MIN_VOLTAGE, pps_capability.maxVoltage);
    if (mosfetState && !psu_in_cc_mode) applyPSUState();
  } else if (current_mode == MODE_PSU && psu_adjust_mode == ADJUST_I) {
    // *** MODIFIED: Use new config parameter ***
    uint16_t large_step = PSU_CONFIG.CURRENT_STEP_MA * PSU_CONFIG.FAST_STEP_MULTIPLIER;
    if (psu_target_current_ma >= large_step) {
      psu_target_current_ma -= large_step;
    } else {
      psu_target_current_ma = 0;
    }
  } else if (current_mode == MODE_PPS) {
    current_requested_pps_voltage -= (PPS_CONFIG.STEP_VOLTAGE * PPS_CONFIG.FAST_STEP_MULTIPLIER);
    current_requested_pps_voltage = constrain(current_requested_pps_voltage, pps_capability.minVoltage, pps_capability.maxVoltage);
    if (mosfetState) requestPPSVoltage(current_requested_pps_voltage, PPS_CONFIG.REQUEST_CURRENT_MA);
  } else if (current_mode == MODE_AVS) {
    current_requested_avs_voltage -= (AVS_CONFIG.STEP_VOLTAGE * AVS_CONFIG.FAST_STEP_MULTIPLIER);
    current_requested_avs_voltage = constrain(current_requested_avs_voltage, avs_capability.minVoltage, avs_capability.maxVoltage);
    if (mosfetState) requestAVSVoltage(current_requested_avs_voltage, AVS_CONFIG.REQUEST_CURRENT_MA);
  }
  settings_changed = true;
}

void handleContinuousPress() {
  unsigned long now = millis();
  bool fast_step_active = false;
  if (nextButton.debouncedState && prevButton.debouncedState) return;

  // *** MODIFIED: Use mode-specific interval ***
  unsigned long interval = 0;
  switch(current_mode) {
    case MODE_PSU:
      interval = PSU_CONFIG.FAST_STEP_INTERVAL_MS;
      break;
    case MODE_PPS:
      interval = PPS_CONFIG.FAST_STEP_INTERVAL_MS;
      break;
    case MODE_AVS:
      interval = AVS_CONFIG.FAST_STEP_INTERVAL_MS;
      break;
    default:
      return; // No continuous press for fixed mode
  }

  if (nextButton.isPressing && nextButton.longPressHandled) {
    if (now - lastContinuousStepTime > interval) {
      onNextFastPressAction();
      lastContinuousStepTime = now;
      fast_step_active = true;
    }
  } else if (prevButton.isPressing && prevButton.longPressHandled) {
    if (now - lastContinuousStepTime > interval) {
      onPrevFastPressAction();
      lastContinuousStepTime = now;
      fast_step_active = true;
    }
  }
  if (fast_step_active) {
    updateDisplay();
  }
}

//==============================================================================
// --- STATE, I2C, SENSORS, AND DISPLAY ---
//==============================================================================
void saveStateToEEPROM() {
  EEPROM.put(EEP::ADDR_MAGIC, CONFIG.EEPROM_MAGIC_NUMBER);
  EEPROM.put(EEP::ADDR_MODE, (uint8_t)current_mode);
  EEPROM.put(EEP::ADDR_FIXED_IDX, (uint8_t)current_profile_index);
  EEPROM.put(EEP::ADDR_PPS_VOLTAGE, current_requested_pps_voltage);
  EEPROM.put(EEP::ADDR_AVS_VOLTAGE, current_requested_avs_voltage);
  EEPROM.put(EEP::ADDR_MOSFET_STATE, (uint8_t)mosfetState);
  EEPROM.put(EEP::ADDR_PSU_TARGET_V, psu_target_voltage);
  EEPROM.put(EEP::ADDR_PSU_TARGET_I, psu_target_current_ma);
}

void writeI2C8(uint8_t d_addr, uint8_t r_addr, uint8_t val) {
  Wire.beginTransmission(d_addr);
  Wire.write(r_addr);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t readI2C8(uint8_t d_addr, uint8_t r_addr) {
  Wire.beginTransmission(d_addr);
  Wire.write(r_addr);
  if (Wire.endTransmission(false) != 0) return 0xFF;
  Wire.requestFrom(d_addr, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0xFF;
}

void recoverI2C() {
  pinMode(SDA, OUTPUT);
  digitalWrite(SDA, HIGH);
  for (int i = 0; i < 10; i++) {
    digitalWrite(SCL, HIGH);
    delayMicroseconds(5);
    digitalWrite(SCL, LOW);
    delayMicroseconds(5);
  }
  digitalWrite(SDA, LOW);
  delayMicroseconds(5);
  digitalWrite(SCL, HIGH);
  delayMicroseconds(5);
  digitalWrite(SDA, HIGH);
  delayMicroseconds(5);
  Wire.begin();
  ina228.begin(I2C_ADDR.INA228);
  configureINA228();
}

void readAllSensors() {
  if (!ina228.conversionReady()) {
    return;
  }
  float rawVoltage = ina228.readBusVoltage();
  if (!isnan(rawVoltage) && rawVoltage >= 0 && rawVoltage < 50.0) { // Increased range for 48V
    currentReadings.voltage = rawVoltage;
    currentReadings.current = ina228.getCurrent_mA();
    currentReadings.power = ina228.getPower_mW();
    currentReadings.temperature = ina228.readDieTemp();
  }
}

void updateDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  if (no_pd_source_flag) {
    display.setTextSize(2);
    display.setCursor(10, 15);
    display.print(F("! NO PD !"));
    display.setTextSize(1);
    display.setCursor(18, 40);
    display.print(F("Basic USB Source"));
    display.display();
    return;
  }

  if (current_mode == MODE_PSU) {
    display.setCursor(0, 0);
    display.print(F("PSU"));
    display.setCursor(30, 0);
    display.print(mosfetState ? F("ON") : F("OFF"));
    display.setCursor(55,0);
    if (psu_in_cc_mode) {
      display.print(psu_cc_is_limited ? F("CC LIM") : F("CC"));
    } else {
      display.print(psu_cv_is_limited ? F("CV LIM") : F("CV"));
    }
    display.setCursor(98, 0);
    if (isnan(currentReadings.temperature)) {
      display.print(F("--C"));
    } else {
      display.print(currentReadings.temperature, 0);
      display.print((char)247);
      display.print(F("C"));
    }
    display.drawFastHLine(0, 10, LAYOUT.SCREEN_WIDTH, SSD1306_WHITE);
    display.setCursor(LAYOUT.LABEL_X, 14);
    display.print(F("Set V:"));
    display.setCursor(LAYOUT.VALUE_X, 14);
    display.print(psu_target_voltage, 2);
    display.print(F("V"));
    display.setCursor(LAYOUT.LABEL_X, 26);
    display.print(F("Set I:"));
    display.setCursor(LAYOUT.VALUE_X, 26);
    display.print(psu_target_current_ma / 1000.0f, 3);
    display.print(F("A"));
    if (psu_adjust_mode == ADJUST_V) {
      display.drawFastHLine(LAYOUT.VALUE_X, 23, 42, SSD1306_WHITE);
    } else {
      display.drawFastHLine(LAYOUT.VALUE_X, 35, 48, SSD1306_WHITE);
    }
    display.drawFastHLine(0, 38, LAYOUT.SCREEN_WIDTH, SSD1306_WHITE);
    display.setCursor(LAYOUT.LABEL_X, 42);
    display.print(F("Load:"));
    float max_amps_psu = PSU_CONFIG.MAX_CURRENT_MA / 1000.0f;
    if (max_amps_psu > 0.01 && !isnan(currentReadings.current)) {
      float measured_amps = fabs(currentReadings.current / 1000.0f);
      float load_percentage = constrain(measured_amps / max_amps_psu, 0.0, 1.0);
      const int bar_x = 36;
      const int bar_width = 88;
      int filled_width = (int)(load_percentage * bar_width);
      display.drawRect(bar_x, 41, bar_width, 8, SSD1306_WHITE);
      if (filled_width > 0) {
        display.fillRect(bar_x, 41, filled_width, 8, SSD1306_WHITE);
      }
    }
    display.setCursor(LAYOUT.LABEL_X, 54);
    if (isnan(currentReadings.voltage)) {
      display.print(F("--.--V"));
    } else {
      display.print(fabs(currentReadings.voltage), 2);
      display.print(F("V"));
    }
    display.setCursor(50, 54);
    if (isnan(currentReadings.current)) {
      display.print(F("-.-A"));
    } else {
      display.print(fabs(currentReadings.current / 1000.0f), 3);
      display.print(F("A"));
    }
    display.setCursor(92, 54);
    if (isnan(currentReadings.power)) {
      display.print(F("--.--W"));
    } else {
      display.print(fabs(currentReadings.power / 1000.0f), 2);
      display.print(F("W"));
    }
  } else {
    display.setCursor(0, 0);
    if (current_mode == MODE_FIXED_PD) display.print(F("FIXED"));
    else if (current_mode == MODE_PPS) display.print(F("PPS"));
    else if (current_mode == MODE_AVS) display.print(F("AVS"));

    display.setCursor(48, 0);
    display.print(F("FET:"));
    display.print(mosfetState ? F("ON") : F("OFF"));
    display.setCursor(98, 0);
    if (isnan(currentReadings.temperature)) {
      display.print(F("--C"));
    } else {
      display.print(currentReadings.temperature, 0);
      display.print((char)247);
      display.print(F("C"));
    }
    display.drawFastHLine(0, 10, LAYOUT.SCREEN_WIDTH, SSD1306_WHITE);
    display.setCursor(LAYOUT.LABEL_X, 14);
    display.print(F("REQ:"));
    display.setCursor(LAYOUT.VALUE_X, 14);

    float max_amps = 0;
    if (current_mode == MODE_FIXED_PD) {
      if (num_supported_profiles > 0) {
        display.print(supported_profiles[current_profile_index].name);
        display.print(F(" / "));
        max_amps = supported_profiles[current_profile_index].max_current;
        display.print(max_amps, 1);
        display.print(F("A"));
      } else {
        display.print(F("5.0V / ?.nA"));
      }
    } else if (current_mode == MODE_PPS) {
      display.print(current_requested_pps_voltage, 2);
      display.print(F("V / "));
      max_amps = pps_capability.maxCurrent;
      display.print(max_amps, 1);
      display.print(F("A"));
    } else if (current_mode == MODE_AVS) {
      display.print(current_requested_avs_voltage, 2);
      display.print(F("V / "));
      max_amps = avs_capability.maxCurrent;
      display.print(max_amps, 1);
      display.print(F("A"));
    }

    display.setCursor(LAYOUT.LABEL_X, 26);
    display.print(F("VBUS:"));
    display.setCursor(LAYOUT.VALUE_X, 26);
    if (isnan(currentReadings.voltage)) {
      display.print(F("---.--V"));
    } else {
      display.print(fabs(currentReadings.voltage), 2);
      display.print(F("V"));
    }
    display.setCursor(LAYOUT.VALUE_X, 38);
    if (isnan(currentReadings.current)) {
      display.print(F("-.---A"));
    } else {
      display.print(fabs(currentReadings.current / 1000.0f), 3);
      display.print(F("A"));
    }
    display.setCursor(LAYOUT.VALUE2_X + 4, 38);
    if (isnan(currentReadings.power)) {
      display.print(F("---.--W"));
    } else {
      display.print(fabs(currentReadings.power / 1000.0f), 2);
      display.print(F("W"));
    }
    display.setCursor(0, 52);
    display.print(F("LOAD"));
    if (max_amps > 0.01 && !isnan(currentReadings.current)) {
      float measured_amps = fabs(currentReadings.current / 1000.0f);
      float load_percentage = constrain(measured_amps / max_amps, 0.0, 1.0);
      int filled_width = (int)(load_percentage * LAYOUT.BAR_WIDTH);
      display.drawRect(LAYOUT.BAR_X, LAYOUT.BAR_Y, LAYOUT.BAR_WIDTH, LAYOUT.BAR_HEIGHT, SSD1306_WHITE);
      if (filled_width > 0) {
        display.fillRect(LAYOUT.BAR_X, LAYOUT.BAR_Y, filled_width, LAYOUT.BAR_HEIGHT, SSD1306_WHITE);
      }
    }
  }
  display.display();
}


//==============================================================================
// --- SYSTEM & FAILSAFES ---
//==============================================================================
void checkFailsafes() {
  if (!isnan(currentReadings.temperature) && currentReadings.temperature > CONFIG.MAX_SAFE_TEMPERATURE_C) {
    halt_system(HALT_OVER_TEMP);
  }
  const float over_voltage_threshold = CONFIG.MAX_SAFE_VOLTAGE + 2.0f; // More tolerance
  if (!isnan(currentReadings.voltage) && currentReadings.voltage > over_voltage_threshold) {
    delay(20);
    float confirmation_voltage = ina228.readBusVoltage();
    if (!isnan(confirmation_voltage) && confirmation_voltage > over_voltage_threshold) {
      halt_system(HALT_OVER_VOLTAGE);
    }
  }

    if (fabs(currentReadings.voltage) < 0.1 && fabs(currentReadings.current) < 1.0) {
      delay(50);
      readAllSensors();
      if (fabs(currentReadings.voltage) < 0.1) {
        halt_system(HALT_SENSOR_FAILURE);
      }
    }
  }


void halt_system(ErrorCondition reason) {
  wdt_disable();
  digitalWrite(PINS.MOSFET, LOW);
  mosfetState = false;
  int blinks = 0;
  switch (reason) {
    case HALT_HUSB_FAULT:   blinks = 1; break;
    case HALT_INA_FAULT:    blinks = 2; break;
    case HALT_OLED_FAULT:   blinks = 3; break;
    case HALT_OVER_TEMP:    blinks = 4; break;
    case HALT_OVER_VOLTAGE: blinks = 5; break;
    case HALT_SENSOR_FAILURE: blinks = 6; break;
    default:                blinks = 10;
  }
  pinMode(PINS.INDICATOR_LED, OUTPUT);
  while (1) {
    for (int i = 0; i < blinks; i++) {
      digitalWrite(PINS.INDICATOR_LED, HIGH);
      delay(150);
      digitalWrite(PINS.INDICATOR_LED, LOW);
      delay(150);
    }
    delay(1000);
  }
}
