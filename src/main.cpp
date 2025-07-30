#include <Arduino.h>
#include <ESP32Servo.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <ArduinoJson.h>
#include <Preferences.h>

Preferences preferences;

// BLE settings
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914c"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a9"

#define NUM_SERVOS 8
#define NUM_POSES 8

// Servo pins - adjust these based on your wiring
const int SERVO_PINS[NUM_SERVOS] = {2, 4, 5, 12, 13, 14, 16, 17};

bool deviceConnected = false;
bool oldDeviceConnected = false;
BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristic = NULL;

unsigned long lastSaveTime = 0;
const unsigned long SAVE_INTERVAL = 5000; // 5 seconds between saves
bool needsSave = false;

struct ServoConfig {
    int position = 90; // 0-180 degrees
};

struct Pose {
    String name;
    ServoConfig servos[NUM_SERVOS];

    Pose() : name("New Pose") {
        for (int i = 0; i < NUM_SERVOS; i++) {
            servos[i].position = 90; // Default to middle position
        }
    }
};

struct Config {
    Pose poses[NUM_POSES];
    int currentPose = 0;
} currentConfig;

Servo servos[NUM_SERVOS];

void saveConfig() {
    unsigned long currentTime = millis();

    // Set flag that we need to save
    needsSave = true;

    // Check if enough time has passed since last save
    if (currentTime - lastSaveTime < SAVE_INTERVAL) {
        return; // Exit if not enough time has passed
    }

    // Reset flag and update last save time
    needsSave = false;
    lastSaveTime = currentTime;

    // Use try-catch to prevent crashes
    try {
        preferences.begin("servomask", false);

        // Save current pose
        preferences.putInt("currentPose", currentConfig.currentPose);

        // Save each pose configuration
        for (int poseIndex = 0; poseIndex < NUM_POSES; poseIndex++) {
            char keyName[20];
            sprintf(keyName, "pose_%d_name", poseIndex);
            preferences.putString(keyName, currentConfig.poses[poseIndex].name);

            for (int servoIndex = 0; servoIndex < NUM_SERVOS; servoIndex++) {
                char keyPos[30];
                sprintf(keyPos, "pose_%d_servo_%d", poseIndex, servoIndex);
                preferences.putInt(keyPos, currentConfig.poses[poseIndex].servos[servoIndex].position);
            }

            // Give some time to the system
            yield();
        }

        preferences.end();
        Serial.println("Config saved!");
    } catch (...) {
        Serial.println("Error saving config");
        preferences.end(); // Make sure we clean up
    }
}

void loadConfig() {
    preferences.begin("servomask", true); // true = readonly

    // Load current pose
    currentConfig.currentPose = preferences.getInt("currentPose", 0);

    // Load each pose configuration
    for (int poseIndex = 0; poseIndex < NUM_POSES; poseIndex++) {
        char keyName[20];
        sprintf(keyName, "pose_%d_name", poseIndex);
        currentConfig.poses[poseIndex].name = preferences.getString(keyName, "Pose " + String(poseIndex + 1));

        for (int servoIndex = 0; servoIndex < NUM_SERVOS; servoIndex++) {
            char keyPos[30];
            sprintf(keyPos, "pose_%d_servo_%d", poseIndex, servoIndex);
            currentConfig.poses[poseIndex].servos[servoIndex].position = preferences.getInt(keyPos, 90); // default to middle position
        }
    }

    preferences.end();
    Serial.println("Config loaded!");
}

void updateServos() {
    int poseIndex = currentConfig.currentPose;
    if (poseIndex < 0 || poseIndex >= NUM_POSES) return;

    for (int i = 0; i < NUM_SERVOS; i++) {
        int position = currentConfig.poses[poseIndex].servos[i].position;
        position = constrain(position, 0, 180); // Ensure position is within valid range
        servos[i].write(position);
    }
}

class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer *pServer) {
        deviceConnected = true;
        Serial.println("Device connected");
    };

    void onDisconnect(BLEServer *pServer) {
        deviceConnected = false;
        Serial.println("Device disconnected");
    }
};

String getConfigJson() {
    StaticJsonDocument<2048> doc; // Increased size for servo config

    // Add current pose
    doc["currentPose"] = currentConfig.currentPose;

    // Add poses array
    JsonArray posesArray = doc.createNestedArray("poses");
    for (int poseIndex = 0; poseIndex < NUM_POSES; poseIndex++) {
        JsonObject pose = posesArray.createNestedObject();
        pose["name"] = currentConfig.poses[poseIndex].name;

        JsonArray servosArray = pose.createNestedArray("servos");
        for (int servoIndex = 0; servoIndex < NUM_SERVOS; servoIndex++) {
            JsonObject servo = servosArray.createNestedObject();
            servo["position"] = currentConfig.poses[poseIndex].servos[servoIndex].position;
        }
    }

    String jsonString;
    serializeJson(doc, jsonString);
    return jsonString;
}

void handlePoseCommand(const String &command) {
    // Expected format: POSE:poseIndex:servo0:servo1:...:servo7
    int firstColon = command.indexOf(':');
    if (firstColon == -1) {
        Serial.println("Invalid pose command format");
        return;
    }

    int secondColon = command.indexOf(':', firstColon + 1);
    if (secondColon == -1) {
        Serial.println("Invalid pose command format");
        return;
    }

    // Parse pose index
    int poseIndex = command.substring(firstColon + 1, secondColon).toInt();
    if (poseIndex < 0 || poseIndex >= NUM_POSES) {
        Serial.println("Invalid pose index: " + String(poseIndex));
        return;
    }

    // Parse servo positions
    String positionsStr = command.substring(secondColon + 1);
    int servoIndex = 0;
    int lastColon = 0;

    while (servoIndex < NUM_SERVOS) {
        int nextColon = positionsStr.indexOf(':', lastColon);
        String positionStr;

        if (nextColon == -1) {
            // Last servo
            positionStr = positionsStr.substring(lastColon);
        } else {
            positionStr = positionsStr.substring(lastColon, nextColon);
        }

        int position = positionStr.toInt();
        position = constrain(position, 0, 180);

        currentConfig.poses[poseIndex].servos[servoIndex].position = position;

        servoIndex++;
        if (nextColon == -1) break;
        lastColon = nextColon + 1;
    }

    // Update current pose and apply it
    currentConfig.currentPose = poseIndex;
    updateServos();
    needsSave = true;

    Serial.println("Applied pose " + String(poseIndex));
}

void handleServoCommand(const String &command) {
    // Expected format: SERVO:servoIndex:position
    int firstColon = command.indexOf(':');
    int secondColon = command.indexOf(':', firstColon + 1);

    if (firstColon == -1 || secondColon == -1) {
        Serial.println("Invalid servo command format");
        return;
    }

    int servoIndex = command.substring(firstColon + 1, secondColon).toInt();
    int position = command.substring(secondColon + 1).toInt();

    if (servoIndex < 0 || servoIndex >= NUM_SERVOS) {
        Serial.println("Invalid servo index: " + String(servoIndex));
        return;
    }

    position = constrain(position, 0, 180);

    // Update current pose
    int poseIndex = currentConfig.currentPose;
    currentConfig.poses[poseIndex].servos[servoIndex].position = position;

    // Apply immediately
    servos[servoIndex].write(position);
    needsSave = true;

    Serial.println("Servo " + String(servoIndex) + " set to " + String(position));
}

class MyCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        std::string value = pCharacteristic->getValue();
        if (value.length() > 0) {
            String command = String(value.c_str());
            Serial.println("Received command: " + command);

            if (command == "GET_CONFIG") {
                String config = getConfigJson();
                Serial.println("Sending config: " + config);
                pCharacteristic->setValue(config.c_str());
                pCharacteristic->notify();
            } else if (command.startsWith("POSE:")) {
                handlePoseCommand(command);
            } else if (command.startsWith("SERVO:")) {
                handleServoCommand(command);
            } else {
                Serial.println("Unknown command: " + command);
            }
        }
    }
};

void setup() {
    Serial.begin(115200);
    Serial.println("Starting Servo Control...");

    loadConfig();

    // Initialize servos
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);

    for (int i = 0; i < NUM_SERVOS; i++) {
        servos[i].setPeriodHertz(50); // Standard 50hz servo
        servos[i].attach(SERVO_PINS[i], 500, 2400); // Min and max pulse width
        servos[i].write(90); // Start at middle position
        delay(100); // Small delay between servo initializations
    }

    // Apply the current pose
    updateServos();

    // Initialize BLE
    BLEDevice::init("Servo Mask");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    BLEService *pService = pServer->createService(SERVICE_UUID);
    pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_WRITE |
        BLECharacteristic::PROPERTY_NOTIFY
    );

    pCharacteristic->addDescriptor(new BLE2902()); // Required for notifications
    pCharacteristic->setCallbacks(new MyCallbacks());
    pService->start();

    // Start advertising
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);
    pAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();

    Serial.println("BLE Servo Control Ready!");
    Serial.println("Device name: Servo Mask");
}

void loop() {
    // Handle BLE connection status
    if (!deviceConnected && oldDeviceConnected) {
        delay(500); // Give the bluetooth stack time to get ready
        pServer->startAdvertising(); // Restart advertising
        Serial.println("Restarting advertising");
        oldDeviceConnected = deviceConnected;
    }

    // Connection established
    if (deviceConnected && !oldDeviceConnected) {
        oldDeviceConnected = deviceConnected;
        Serial.println("New device connected");
    }

    // Check if we need to save and enough time has passed
    if (needsSave && (millis() - lastSaveTime >= SAVE_INTERVAL)) {
        saveConfig();
    }

    delay(20); // Small delay to prevent overwhelming the system
}