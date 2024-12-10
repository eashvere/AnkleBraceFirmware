#include <Arduino.h>
#include <Wire.h>
#include <ArduinoBLE.h>
#include <driver/adc.h>
#include "esp_bt.h"
#include <EEPROM.h>

#define DEBUG_MODE

#ifdef DEBUG_MODE
  #define DEBUG_PRINTLN(x) Serial.println(x)
  #define DEBUG_PRINT(x) Serial.print(x)
  #define DEBUG_BEGIN(baud) Serial.begin(baud)
#else
  #define DEBUG_PRINTLN(x) // Do nothing
  #define DEBUG_PRINT(x)   // Do nothing
  #define DEBUG_BEGIN(baud) // Do nothing
#endif

#define SECONDS_TO_MILLISECONDS(s) ((s) * 1000UL)
#define EEPROM_SIZE 12 // 3 ints (thresholds) is 4 bytes * 3 = 12

#define CHARACTERISTIC_SIZE  22// Change based on your requirement
BLEService braceBLEService("23e6ab8d-4bf2-4be6-b3e0-9d4c07028e5e");  // 1816 is the defined UUID for cycling tech...
BLEStringCharacteristic bendSensorData("bba3b2e6-5dbf-406b-ad70-072c8f46b5cf",  // Custom characteristic UUID
                                   BLERead | BLENotify | BLEWrite,
                                   CHARACTERISTIC_SIZE);  // Characteristic value length
BLEDescriptor bendSensorDescription("f827a712-962e-4886-837b-2a1eb6097315", "Bend Sensors");  // Used for enabling notifications.

BLEIntCharacteristic calibrationStarter("6ccb6d2d-53f8-4e4c-9e83-4245cfa668ef",  // Custom characteristic UUID
                                   BLEWrite | BLERead | BLENotify);
BLEDescriptor calibrationDescription("bcefb9eb-be4e-4f95-855c-7b0a10c8826d", "Calibration");


/***** FLEX SENSOR PINS *****/
#define LEFT_FLEX_SENSOR A0
#define FRONT_FLEX_SENSOR A3
#define RIGHT_FLEX_SENSOR A6

int left_threshold = 375;
int front_threshold = 600;
int right_threshold = 430;

/***** MPU INTERRUPT VARIABLES ******/
#define SIGNAL_PATH_RESET  0x68
#define I2C_SLV0_ADDR      0x37
#define ACCEL_CONFIG       0x1C
#define MOT_THR            0x1F  // Motion detection threshold bits [7:0]
#define MOT_DUR            0x20  // This seems wrong // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define MOT_DETECT_CTRL    0x69
#define INT_ENABLE         0x38
#define PWR_MGMT           0x6B //SLEEPY TIME
#define INT_STATUS 0x3A
#define MPU6050_ADDRESS 0x68 //AD0 is 0

#define TIME_BEFORE_SLEEP SECONDS_TO_MILLISECONDS(100000)
#define MPU_INT_PIN GPIO_NUM_15
#define MPU_SENS 70
#define VELOCITY_THRESHOLD 100

unsigned int sleepStartTimer = millis();

void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
  Wire.begin();
  Wire.beginTransmission(address);  // Initialize the Tx buffer
  Wire.write(subAddress);           // Put slave register address in Tx buffer
  Wire.write(data);                 // Put data in Tx buffer
  Wire.endTransmission();           // Send the Tx buffer
}

// sens argument configures wake up sensitivity
void configureMPU(int sens){
  writeByte( MPU6050_ADDRESS, 0x6B, 0x00);
  writeByte( MPU6050_ADDRESS, SIGNAL_PATH_RESET, 0x07);//Reset all internal signal paths in the MPU-6050 by writing 0x07 to register 0x68;
  // writeByte( MPU6050_ADDRESS, I2C_SLV0_ADDR, 0x20);//write register 0x37 to select how to use the interrupt pin. For an active high, push-pull signal that stays until register (decimal) 58 is read, write 0x20.
  writeByte( MPU6050_ADDRESS, ACCEL_CONFIG, 0x01);//Write register 28 (==0x1C) to set the Digital High Pass Filter, bits 3:0. For example set it to 0x01 for 5Hz. (These 3 bits are grey in the data sheet, but they are used! Leaving them 0 means the filter always outputs 0.)
  writeByte( MPU6050_ADDRESS, MOT_THR, sens);  //Write the desired Motion threshold to register 0x1F (For example, write decimal 20).
  writeByte( MPU6050_ADDRESS, MOT_DUR, 40 );  //Set motion detect duration to 1  ms; LSB is 1 ms @ 1 kHz rate
  writeByte( MPU6050_ADDRESS, MOT_DETECT_CTRL, 0x15); //to register 0x69, write the motion detection decrement and a few other settings (for example write 0x15 to set both free-fall and motion decrements to 1 and accelerometer start-up delay to 5ms total by adding 1ms. )
  writeByte( MPU6050_ADDRESS, 0x37, 140 ); // now INT pin is active low
  writeByte( MPU6050_ADDRESS, INT_ENABLE, 0x40 ); //write register 0x38, bit 6 (0x40), to enable motion detection interrupt.
  writeByte( MPU6050_ADDRESS, PWR_MGMT, 8 ); // 101000 - Cycle & disable TEMP SENSOR
  writeByte( MPU6050_ADDRESS, 0x6C, 7); // Disable Gyros
}


void start_bluetooth() {
  // Initialize BLE hardware
  if (!BLE.begin()) {
    while (1) {
      DEBUG_PRINTLN("Starting BLE failed!");
      delay(1000);
    }
  }

  // Set the local name and service information
  BLE.setLocalName("SmartAnkleBrace");
  BLE.setAdvertisedService(braceBLEService);
  // Add custom characteristic
  braceBLEService.addCharacteristic(bendSensorData);
  bendSensorData.addDescriptor(bendSensorDescription);

  braceBLEService.addCharacteristic(calibrationStarter);
  calibrationStarter.addDescriptor(calibrationDescription);

  BLE.addService(braceBLEService);

  calibrationStarter.writeValue(false);

  // Start advertising
  BLE.advertise();
  DEBUG_PRINTLN("Bluetooth device active, waiting for connections...");
}

void setup() {
  // Open Serial
  #ifdef DEBUG_MODE
    Serial.begin(115200);
    while(!Serial);
  #endif

  //EEPROM.begin(EEPROM_SIZE);
  Wire.begin();
  start_bluetooth();

  configureMPU(MPU_SENS); // Setup MPU for interrupt, power down gyros & temp sensor

  // EEPROM.get(0, left_threshold);
  // EEPROM.get(4, front_threshold);
  // EEPROM.get(8, right_threshold);

  DEBUG_PRINTLN("Left Threshold:");
  DEBUG_PRINTLN(left_threshold);
  DEBUG_PRINTLN("Front Threshold:");
  DEBUG_PRINTLN(front_threshold);
  DEBUG_PRINTLN("Right Threshold:");
  DEBUG_PRINTLN(right_threshold);

  sleepStartTimer = millis();
}

int flex_left, flex_front, flex_right;

void read_flex_sensor() {
  flex_left = analogRead(LEFT_FLEX_SENSOR);
  flex_front = analogRead(FRONT_FLEX_SENSOR);
  flex_right = analogRead(RIGHT_FLEX_SENSOR);
}

BLEDevice central;

int left_normal, front_normal, right_normal;
int left_min, front_min, right_min;

unsigned int startCalibrationTime = millis();
bool ongoingCalibration = false;

int calibrationCounts = 0;

void calibration() {
  if (!central) {
    calibrationStarter.writeValue(0);
  }

  if (!ongoingCalibration) {
    calibrationCounts = 0;
    startCalibrationTime = millis();
    return;
  }
  if (millis() - startCalibrationTime < SECONDS_TO_MILLISECONDS(0.2)) {
    DEBUG_PRINTLN("Start Calibration");
    left_normal = flex_left;
    front_normal = flex_front;
    right_normal = flex_right;
    calibrationCounts = 1;
  } else if (millis() - startCalibrationTime < SECONDS_TO_MILLISECONDS(10)) {
    DEBUG_PRINTLN("FIND NORMAL");
    left_normal = ((left_normal * calibrationCounts) + flex_left) / (calibrationCounts + 1);
    front_normal = ((front_normal * calibrationCounts) + flex_front) / (calibrationCounts + 1);
    right_normal = ((right_normal * calibrationCounts) + flex_right) / (calibrationCounts + 1);
  } else if (millis() - startCalibrationTime < SECONDS_TO_MILLISECONDS(10.2)) {
    calibrationStarter.writeValue(2);
    left_min = 10000;
    front_min = 10000;
    right_min = 10000;
  } else if (millis() - startCalibrationTime < SECONDS_TO_MILLISECONDS(20)) {
    DEBUG_PRINTLN("FIND MIN");
    left_min = min(left_min, flex_left);
    front_min = min(front_min, flex_front);
    right_min = min(right_min, flex_right);
  } else if (millis() - startCalibrationTime < SECONDS_TO_MILLISECONDS(21)) {
    left_threshold = left_normal - ((left_normal - left_min) * 0.5);
    front_threshold = front_normal - ((front_normal - front_min) * 0.5);
    right_threshold = right_normal - ((right_normal - right_min) * 0.5);
  } else if (millis() - startCalibrationTime > SECONDS_TO_MILLISECONDS(21)) {
    DEBUG_PRINTLN("Left:");
    DEBUG_PRINTLN(left_normal);
    DEBUG_PRINTLN(left_min);
    DEBUG_PRINTLN(left_threshold);
    DEBUG_PRINTLN("Front:");
    DEBUG_PRINTLN(front_normal);
    DEBUG_PRINTLN(front_min);
    DEBUG_PRINTLN(front_threshold);
    DEBUG_PRINTLN("Right:");
    DEBUG_PRINTLN(right_normal);
    DEBUG_PRINTLN(right_min);
    DEBUG_PRINTLN(right_threshold);

    // EEPROM.put(0, left_threshold);
    // EEPROM.put(4, front_threshold);
    // EEPROM.put(8, right_threshold);
    // EEPROM.commit();

    calibrationStarter.writeValue(3);

    ongoingCalibration = false;
  }
}


bool front_active = false;
bool left_active = false;
bool right_active = false;

void send_data_over_ble() {
  if (!central) {
    //DEBUG_PRINTLN("Waiting to connect to central.");
    return;
  }

  if (central.connected()) {
    DEBUG_PRINT("Left Sensor:");
    DEBUG_PRINTLN(flex_left);
    DEBUG_PRINT("Front Sensor:");
    DEBUG_PRINTLN(flex_front);
    DEBUG_PRINT("Right Sensor:");
    DEBUG_PRINTLN(flex_right);

    // Send data over BLE
    char message[CHARACTERISTIC_SIZE];
    memset(message, '\0', sizeof(message));

    if (flex_left <= left_threshold) {
      if (!left_active) {
        snprintf(message, sizeof(message), "L,%d %d %d", flex_left, flex_front, flex_right);
        bendSensorData.writeValue(message);
        left_active = true;

        DEBUG_PRINT("Left Threshold");
        DEBUG_PRINTLN(left_threshold);
        DEBUG_PRINT("Sent message: ");
        DEBUG_PRINTLN(message);
      }
    } else {
      if (left_active) {
        snprintf(message, sizeof(message), "SL,%d %d %d", flex_left, flex_front, flex_right);
        bendSensorData.writeValue(message);
        left_active = false;
        
        DEBUG_PRINT("Sent message: ");
        DEBUG_PRINTLN(message);
      }
    }

    if (flex_front <= front_threshold) {
      if (!front_active) {
        snprintf(message, sizeof(message), "F,%d %d %d", flex_left, flex_front, flex_right);
        bendSensorData.writeValue(message);
        front_active = true;

        DEBUG_PRINT("Front Threshold");
        DEBUG_PRINTLN(front_threshold);
        DEBUG_PRINT("Sent message: ");
        DEBUG_PRINTLN(message);
      }
    } else {
      if (front_active) {
        snprintf(message, sizeof(message), "SF,%d %d %d", flex_left, flex_front, flex_right);
        bendSensorData.writeValue(message);
        front_active = false;

        DEBUG_PRINT("Sent message: ");
        DEBUG_PRINTLN(message);
      }
    }

    if (flex_right <= right_threshold) {
      if (!right_active) {
        snprintf(message, sizeof(message), "R,%d %d %d", flex_left, flex_front, flex_right);
        bendSensorData.writeValue(message);
        right_active = true;

        DEBUG_PRINT("Right Threshold");
        DEBUG_PRINTLN(right_threshold);
        DEBUG_PRINT("Sent message: ");
        DEBUG_PRINTLN(message);
      }
    } else {
      if (right_active) {
        snprintf(message, sizeof(message), "SR,%d %d %d", flex_left, flex_front, flex_right);
        bendSensorData.writeValue(message);
        right_active = false;

        DEBUG_PRINT("Sent message: ");
        DEBUG_PRINTLN(message);
      }
    }
  } else {
    DEBUG_PRINT("Not connected to central: ");
    DEBUG_PRINTLN(central.address());
  }
}

void waitForCalibration() {
  if (!central || ongoingCalibration) {
    return;
  }
  
  if (central.connected()) {
    if (calibrationStarter.written()) {
      DEBUG_PRINTLN("CALIBRATION CONNECTED");
      int value = calibrationStarter.value();
      if (value == 1) {
        ongoingCalibration = true;
        sleepStartTimer = millis();
        startCalibrationTime = millis();
        DEBUG_PRINTLN("Start Calibration");
      }
    }
  } else {
    DEBUG_PRINT("Not connected to central: ");
    DEBUG_PRINTLN(central.address());
  }
}

float lastAccelValue = 0;
float lastAccelRead = millis();

void readMPUAccel() {
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(MPU6050_ADDRESS, 6, true);

  unsigned int newAccelRead = millis();
  float AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
  float AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
  float AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value

  float newAccelValue = abs(AccX) + abs(AccY) + abs(AccZ);
  float velocity = abs(newAccelValue - lastAccelValue) / ((newAccelRead - lastAccelRead) / 1000);

  lastAccelValue = newAccelValue;
  lastAccelRead = newAccelRead;

  if (velocity > VELOCITY_THRESHOLD) {
    sleepStartTimer = millis();
    DEBUG_PRINTLN("Device Moved. Reset sleep timer...");
    DEBUG_PRINT("Velocity: ");
    DEBUG_PRINTLN(velocity);
  }
}

void deepSleepOnInactivity() {
  if (millis() - sleepStartTimer >= TIME_BEFORE_SLEEP) {
    DEBUG_PRINTLN("Time limit reached. Going to sleep...");
    ongoingCalibration = false;

    esp_bt_controller_disable();
    esp_bt_controller_deinit();

    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
    esp_sleep_enable_ext0_wakeup(MPU_INT_PIN, 0);

    esp_deep_sleep_start();
  }
}


void loop() {
  central = BLE.central();

  read_flex_sensor();

  waitForCalibration();
  calibration();

  send_data_over_ble();

  readMPUAccel();

  //DEBUG_PRINTLN(millis() - sleepStartTimer);

  if (!ongoingCalibration) {
    //deepSleepOnInactivity();
  }

  delay(50);
}