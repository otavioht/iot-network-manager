// IoTNetworkManagement.h
#ifndef IoTNetworkManagement_h
#define IoTNetworkManagement_h

// #include "Arduino.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <Preferences.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <map>
#include <functional>
#include <vector>
#include <DHT.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "EmonLib.h"   
#include "esp_task_wdt.h"

class IoTNetworkManagement {

public:
    enum class SensorType {
        ENERGY,
        SOUND,
        TEMPERATURE,
        LIGHT
    };

    static const std::map<SensorType, String> sensorTypeToTopic;

    enum class FailureDetectionType {
        ZERO_VALUE,
        THRESHOLD_VALUE
        
    };

    struct Sensor {
        SensorType type;
        uint8_t pin;
        float threshold;
        FailureDetectionType failureType;
        std::function<float()> readFunction;
        TaskHandle_t taskHandle;
        uint32_t checkInterval;
        String topic;
    };
    DHT* _dht; 
    
    IoTNetworkManagement();
    // IoTNetworkManagement(const char* device_id, const char* mqtt_broker, uint16_t mqtt_port, const char* mqtt_user, const char* mqtt_password);
    static IoTNetworkManagement& getInstance() {
        static IoTNetworkManagement instance;
        return instance;
    }
    void getWifiSignalStrength(); 
    void reconnectMqtt();
    const char* _device_id;
    const char* _mqtt_broker;
    uint16_t _mqtt_port;
    const char* _mqtt_user;
    const char* _mqtt_password;
    PubSubClient _mqttClient;
    EnergyMonitor emon1;  
    SemaphoreHandle_t mqttMutex;
    // IoTNetworkManagement(const IoTNetworkManagement&) = delete; // No copy constructor
    void setupTemperatureSensor(uint8_t pin);
    // MQTT callback to handle incoming messages
    static void mqttCallback(char* topic, byte* payload, unsigned int length);
    void begin();
    // void handle();
    void checkIRSender();
    void updateTime();
    Sensor* findSensorByTopic(const String& topic); 
    // void detectFailure(const Sensor& sensor); 
    void setupIRCheckPins(uint8_t irLedPin, uint8_t irReceiverPin);
    void setupLightSensor(uint8_t pin);
    void setupSoundSensor(uint8_t pin, float threshold = 0.0f);
    void publishData(const char* type_of_data, const char* payload);
    void initialize(const char* device_id, const char* mqtt_broker, uint16_t mqtt_port, const char* mqtt_user, const char* mqtt_password) {
        _device_id = device_id;
        _mqtt_broker = mqtt_broker;
        _mqtt_port = mqtt_port;
        _mqtt_user = mqtt_user;
        _mqtt_password = mqtt_password;
        _mqttClient = PubSubClient(mqtt_broker, mqtt_port, mqttCallback, _wifiClient);
        mqttMutex = xSemaphoreCreateMutex();
        emon1.current(25, 0.50); 
    }

private:
    // Private members
    WiFiClient _wifiClient;

    WiFiUDP _ntpUDP;
    NTPClient _timeClient;
    Preferences _preferences;
    std::vector<Sensor> _sensors;
    unsigned long _lastBytesReceived;
    unsigned long _lastBytesSent;
    unsigned long _lastReportTime;
    unsigned long lastBootTime;
    bool isIRSetupCalled;
    bool isTemperatureSetupCalled;
    bool isSoundSetupCalled;
    bool isLightSetupCalled;
    uint8_t _IR_LED_PIN;
    uint8_t _IR_RECEIVER_PIN;
    uint8_t _SOUND_PIN;
    uint8_t _ENERGY_PIN;
    uint8_t _LIGHT_PIN;
    // Task Handles
    TaskHandle_t _checkIRTaskHandle;
    TaskHandle_t _deviceStatusTaskHandle;
    // TaskHandle_t _networkLoadTaskHandle;
    TaskHandle_t _wifiSignalStrengthTaskHandle;
    TaskHandle_t _errorReportTaskHandle;
    // Private methods
    static void lightSensorTask(void *parameter);
    static void soundSensorTask(void *parameter);
    static void temperatureSensorTask(void *parameter);
    static void deviceStatusTask(void* parameter);
    static void checkIRSenderTask(void* parameter);
    static void wifiSignalStrengthTask(void* parameter);
    // static void networkLoadTask(void* parameter);
    static void errorReportTask(void* parameter);
    static void mqttLoopTask(void* parameter);
    void handleError(const String& error);
    void updateBootCount();
    unsigned long getBootCount();
    void updateLastBootTime();
    String getLastBootTime();

};

#endif
