// IoTNetworkManagement.cpp
#include "IoTNetworkManagement.h"

IoTNetworkManagement::IoTNetworkManagement()
:_device_id(""), 
_mqtt_broker(""), 
_mqtt_port(0), 
_mqtt_user(""), 
_mqtt_password(""), 
_mqttClient(_wifiClient), 
_dht(nullptr),
_timeClient(_ntpUDP, "0.br.pool.ntp.org",  -3 * 3600, 60000){
_sensors = std::vector<Sensor>();
isIRSetupCalled = false;
isTemperatureSetupCalled = false;
isSoundSetupCalled = false;
isLightSetupCalled = false;
}

const std::map<IoTNetworkManagement::SensorType, String> IoTNetworkManagement::sensorTypeToTopic = {
        {SensorType::ENERGY, "energy"},
        {SensorType::SOUND, "sound"},
        {SensorType::TEMPERATURE, "temperature"},
        {SensorType::LIGHT, "light"}
};
void IoTNetworkManagement::begin() {
    // Setup code here: WiFi connection, MQTT connection, etc.
    Serial.println("Connecting to broker...");
    _mqttClient = MqttClient(_wifiClient);
    _mqttClient.connect(_mqtt_broker, _mqtt_port);
    _mqttClient.onMessage(mqttCallback);
    _mqttClient.subscribe("connected_devices");
    _mqttClient.subscribe("esp32/+/commands");

    if (mqttMutex == NULL) {
        Serial.println("Creating mutex");
        mqttMutex = xSemaphoreCreateMutex();
    }
    _mqttClient.poll();
    // Start the IR check task
    if (isIRSetupCalled) {
        Serial.println("IR setup called");
        xTaskCreatePinnedToCore(
            checkIRSenderTask,         
            "checkIRSenderTask",        
            2048,                
            (void*) this,                 
            1,                    
            NULL,                 
            0                  
        );
    }

    xTaskCreatePinnedToCore(
        deviceStatusTask,         
        "DeviceStatusTask",       
        96000,                    
        this,                     
        0,                        
        &_deviceStatusTaskHandle, 
        0                         
    );

    // Start the Wi-Fi signal strength task
    xTaskCreatePinnedToCore(
        wifiSignalStrengthTask,      
        "WifiSignalStrengthTask",    
        4096,                        
        this,                        
        0,                           
        &_wifiSignalStrengthTaskHandle,
        1
    );
    
    xTaskCreatePinnedToCore(
        errorReportTask,              
        "ErrorReportTask",            
        2048,                         
        this,                         
        1,                            
        &_errorReportTaskHandle,
        1    
    );

    xTaskCreatePinnedToCore(
        this->mqttLoopTask, 
        "mqttLoopTask",     
        4096,              
        this,               
        2,                  
        NULL,   
        1               
    );

    Serial.println("tasks created");
    updateBootCount();
    _timeClient.begin();
    updateLastBootTime();

    WiFi.onEvent([this](WiFiEvent_t event, WiFiEventInfo_t info) {
        this->handleError("Wi-Fi disconnected");
    }, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
}

void IoTNetworkManagement::mqttCallback(int messageSize) {
        // Convert payload to a String for easy comparison
        IoTNetworkManagement& instance = IoTNetworkManagement::getInstance();
        Serial.println("Message received");

        String message;

        while (instance._mqttClient.available()) {
            message += (char) instance._mqttClient.read();
        }

        // Check if the message is "GET" and if it's on the "connected_devices" topic
        if (String(instance._mqttClient.messageTopic()) == "connected_devices" && message == "GET") {
            // Respond with "CONNECTED" message
            Serial.println("Connected devices");
            instance.publishData("connected_devices", "CONNECTED");
        } 

}


void IoTNetworkManagement::reconnectMqtt() {
    // Attempt to reconnect to the MQTT broker
    IoTNetworkManagement& instance = IoTNetworkManagement::getInstance();
    if (instance.mqttMutex == NULL) {
        Serial.println("Failed to create semaphore");
        // Handle the error...  
        instance.mqttMutex = xSemaphoreCreateMutex();
    }
    if (xSemaphoreTake(instance.mqttMutex, portMAX_DELAY) == pdTRUE) {
        Serial.println("Trying Connection...");
        instance._mqttClient.connect(_mqtt_broker, _mqtt_port);
        instance._mqttClient.onMessage(mqttCallback);

        Serial.println("MQTT connected");
        // Resubscribe to topics if necessary
        instance._mqttClient.subscribe("connected_devices");
        String topic = String("esp32/") + _device_id + "/+";
        instance._mqttClient.subscribe(topic.c_str());
        instance._mqttClient.poll();

        xSemaphoreGive(instance.mqttMutex);
    } else {
        // Handle the error case where the mutex couldn't be taken
        Serial.println("Error: couldn't take MQTT mutex");  
    }
    

}

void IoTNetworkManagement::publishData(const char* type_of_data, const char* payload) {
    // Construct the MQTT topic
    // if (mqttMutex == NULL) {
    //     Serial.println("Failed to create semaphore");
    //     // Handle the error...  
    //     mqttMutex = xSemaphoreCreateMutex();
    // }
    // if (xSemaphoreTake(mqttMutex, portMAX_DELAY) == pdTRUE) {

        Serial.println("Publishing data...");
        Serial.println(type_of_data);
        Serial.println(payload);
        String topic;

        if(type_of_data == "connected_devices"){
            topic = String("connected_devices");
        } else {
            topic = String("esp32/") + _device_id + "/" + type_of_data;
        }

        if(!_mqttClient.connected()){
            if (!_mqttClient.connect(_mqtt_broker, _mqtt_port)) {
                Serial.print("MQTT connection failed! Error code = ");
                Serial.println();
                handleError("MQTT not connected");
                reconnectMqtt();
            }
        }

        Serial.println("Connected to MQTT broker");

        Serial.println("Trying to publish");
        _mqttClient.beginMessage(topic);
        _mqttClient.print(payload);
        _mqttClient.endMessage();
        Serial.println("Published");


        // Don't forget to give the mutex back when you're done
    //     xSemaphoreGive(mqttMutex);
    // } else {
    //     // Handle the error case where the mutex couldn't be taken
    //     Serial.println("Error: couldn't take MQTT mutex");  
    // }

}

void IoTNetworkManagement::checkIRSenderTask(void *parameter) {
    // Cast the parameter to the correct class instance
    if (parameter == nullptr) {
        Serial.println("Error: parameter is null");
        return;
    }
    auto instance = static_cast<IoTNetworkManagement*>(parameter);

    for (;;) { // Task loop
        float downValue = analogRead(instance->_IR_RECEIVER_PIN);
        Serial.println(downValue); 
        digitalWrite(instance->_IR_LED_PIN, HIGH);
        // vTaskDelay(3000 / portTICK_PERIOD_MS); // IR on for 3 seconds
        float upValue = analogRead(instance->_IR_RECEIVER_PIN);
        // Check if the phototransistor received the IR signal
        Serial.println(upValue);
        if ((downValue - upValue) > 150) {
            Serial.println("IR LED is working.");
            // Send a "working" status via MQTT, or log as necessary
            instance->publishData("air_remote", "Online");
        } else {
            instance->handleError("IR LED check failed!");
            // Send a "failed" status via MQTT, or log as necessary
            instance->publishData("air_remote", "Offline");
        }
        
        digitalWrite(instance->_IR_LED_PIN, LOW);


        double Irms = instance->emon1.calcIrms(1480);
        if(Irms == 0.0f){
            instance->handleError("Failed to read from energy sensor!");
            instance->publishData("energy", "Offline");
        } else {
            instance->publishData("energy", "Online");
        }

        vTaskDelay(3130000 / portTICK_PERIOD_MS); // Wait for 5 minutes
    }
}

void IoTNetworkManagement::setupIRCheckPins(uint8_t irLedPin, uint8_t irReceiverPin) {
    // Assign the pins for the IR LED and the receiver
    _IR_LED_PIN = irLedPin;
    _IR_RECEIVER_PIN = irReceiverPin;

    // Set the pin modes for the IR LED and receiver
    pinMode(_IR_LED_PIN, OUTPUT);
    pinMode(_IR_RECEIVER_PIN, INPUT);
    adcAttachPin(_IR_RECEIVER_PIN); 
    isIRSetupCalled = true;
}

void IoTNetworkManagement::deviceStatusTask(void *parameter) {
    // Cast the parameter to the correct class instance
    if (parameter == nullptr) {
        Serial.println("Error: parameter is null");
        return;
    }
    auto instance = static_cast<IoTNetworkManagement*>(parameter);
    const long interval = 219330 / portTICK_PERIOD_MS;
    esp_task_wdt_delete(NULL);
    for (;;) { // Task loop
        // Get device status and uptime
        String deviceStatus = WiFi.status() == WL_CONNECTED ? "Online" : "Offline";
        unsigned long uptime = (int) millis() / 60000;
    // Inside the task loop, right after publishing Device Uptime
        unsigned long bootCount = instance->getBootCount();
        String lastBootTime = instance->getLastBootTime();

        // Publish device status and uptime
        instance->publishData("status", deviceStatus.c_str());
        instance->publishData("uptime", String(uptime).c_str());
        instance->publishData("boot_count", String(bootCount).c_str());
        instance->publishData("last_boot_time", lastBootTime.c_str());
        // Sleep for the interval
        vTaskDelay(interval);
    }
}

void IoTNetworkManagement::updateBootCount() {
    _preferences.begin("nvs", false); 
    unsigned long bootCount = _preferences.getUInt("bootCount", 0); 
    _preferences.end();
}

unsigned long IoTNetworkManagement::getBootCount() {
    _preferences.begin("nvs", true);
    unsigned long bootCount = _preferences.getUInt("bootCount", 0);
    _preferences.end();
    return bootCount;
}

void IoTNetworkManagement::updateLastBootTime() {
    // Update the NTP client and get the current time
    _timeClient.update();
    lastBootTime = _timeClient.getEpochTime();

    // Save the last boot time to NVS
    _preferences.begin("nvs", false);
    _preferences.putUInt("last_boot_time", (uint32_t)lastBootTime);
    _preferences.end();
}

String IoTNetworkManagement::getLastBootTime() {
    _preferences.begin("nvs", true);
    time_t lastBoot = _preferences.getUInt("last_boot_time", 0);
    _preferences.end();

    if (lastBoot == 0) {
        return String("Unknown");
    } else {
        char timeString[25];
        strftime(timeString, sizeof(timeString), "%Y-%m-%d %H:%M:%S", localtime(&lastBoot));
        return String(timeString);
    }
}

void IoTNetworkManagement::handleError(const String& error) {

    // Log the error to NVS
    _preferences.begin("nvs", false);
    _preferences.putString("last_error", error);
    _preferences.end();
    Serial.print('Error: ');
    Serial.print(error);
    // publish this error to an MQTT topic for error logging    

}


void IoTNetworkManagement::wifiSignalStrengthTask(void *parameter) {
    if (parameter == nullptr) {
        Serial.println("Error: parameter is null");
        return;
    }
    auto instance = static_cast<IoTNetworkManagement*>(parameter);
    esp_task_wdt_delete(NULL);
    for (;;) {
        instance->getWifiSignalStrength();
        vTaskDelay(165440 / portTICK_PERIOD_MS); // Delay for 3 minutes
    }
}

void IoTNetworkManagement::getWifiSignalStrength() {
    long rssi = WiFi.RSSI();
    String signalStrength = String(rssi);
    Serial.println(signalStrength);
    // Publish Wi-Fi signal strength to an MQTT topic
    publishData("signal_strength", signalStrength.c_str());
}

IoTNetworkManagement::Sensor* IoTNetworkManagement::findSensorByTopic(const String& topic) {
    for (auto& sensor : _sensors) {
        if (sensor.topic == topic) {
            return &sensor;
        }
    }
    return nullptr; // Return nullptr if no sensor is found for the topic
}

void IoTNetworkManagement::setupSoundSensor(uint8_t pin, float threshold) {
    // Create and configure the sound sensor object
    _SOUND_PIN = pin;
    isSoundSetupCalled = true;

    // Create the task that will monitor the sound sensor
    xTaskCreatePinnedToCore(
        soundSensorTask,              // Function to implement the task
        "SoundSensorTask",            // Name of the task
        4096,                         // Stack size in words (adjust as necessary)
        this,             // Task input parameter (pointer to the sound sensor)
        3,                            // Priority of the task
        NULL,   // Task handle,
        0
    );
}

// Static task function that monitors the sound sensor
void IoTNetworkManagement::soundSensorTask(void *parameter) {
    if (parameter == nullptr) {
        Serial.println("Error: parameter is null");
        return;
    }
    IoTNetworkManagement* manager = static_cast<IoTNetworkManagement*>(parameter);

    for (;;) {
        // Read the sound level
        float soundLevel = analogRead(manager->_SOUND_PIN);
        String topic = "sound";
        // Check if any reads failed and exit early (to try again).
        if (isnan(soundLevel) || soundLevel < 650) {
            Serial.println(F("Failed to read from sound sensor!"));
            manager->handleError("Failed to read from sound sensor!");
            manager->publishData(topic.c_str(), "Offline");
        } else {
            Serial.println(soundLevel);
            manager->publishData(topic.c_str(), "Online");
        }

        // Delay between reads
        vTaskDelay(131000 / portTICK_PERIOD_MS); 
    }
}

void IoTNetworkManagement::setupTemperatureSensor(uint8_t pin) {
    // Initialize the DHT sensor
    _dht = new DHT(pin, DHT11);
    
    if(_dht == nullptr) return;
    
    _dht->begin();
    isTemperatureSetupCalled = true;
    // Create and start the temperature sensor task
    xTaskCreatePinnedToCore(
        temperatureSensorTask,    // Task function
        "TemperatureSensorTask",  // Name of the task
        8240,                     // Stack size (adjust as necessary)
        this,                     // Task input parameter (passing the IoTNetworkManagement instance)
        1,                        // Priority of the task
        NULL,
        0
    );
}

// In the task where you handle the sensor readings
void IoTNetworkManagement::temperatureSensorTask(void* parameter) {
    if (parameter == nullptr) {
        Serial.println("Error: parameter is null");
        return;
    }
    IoTNetworkManagement* manager = static_cast<IoTNetworkManagement*>(parameter);

    for (;;) {
        // Reading temperature or humidity takes about 250 milliseconds!
        float temperature = manager->_dht->readTemperature();
        // Check if any reads failed and exit early (to try again).
        if (isnan(temperature)) {
            Serial.println(F("Failed to read temperature from DHT sensor!"));
            manager->handleError("Failed to read temperature from DHT sensor!");
            manager->publishData("temperature", "Offline");
        } else {
            Serial.println(temperature);
            manager->publishData("temperature", "Online");
        }

        // Wait a bit before reading again
        vTaskDelay(91000 / portTICK_PERIOD_MS); // Delay between reads (adjust as necessary)
    }
}

void IoTNetworkManagement::setupLightSensor(uint8_t pin) {
    // Configure the GPIO pin for the light sensor
    pinMode(pin, INPUT);    
    _LIGHT_PIN = pin;
    isLightSetupCalled = true;

    // Create and configure the light sensor object
    Sensor lightSensor;
    lightSensor.pin = pin;
    lightSensor.type = SensorType::LIGHT;
    lightSensor.readFunction = [pin]() -> float { return analogRead(pin); }; // Replace with actual light sensor read logic
    lightSensor.topic = sensorTypeToTopic.at(SensorType::LIGHT); // Use the predefined topic for light sensors

    // Add the light sensor to the vector of sensors
    _sensors.push_back(lightSensor);

    // Create the task that will monitor the light sensor
    xTaskCreatePinnedToCore(
        lightSensorTask,              // Function to implement the task
        "LightSensorTask",            // Name of the task
        3480,                         // Stack size in words (adjust as necessary)
        this,             // Task input parameter (pointer to the light sensor)
        2,                            // Priority of the task
        NULL,
        0
    );
}

// Static task function that monitors the light sensor
void IoTNetworkManagement::lightSensorTask(void *parameter) {
    if (parameter == nullptr) {
        Serial.println("Error: light parameter is null");
        return;
    }
    IoTNetworkManagement* manager = static_cast<IoTNetworkManagement*>(parameter);
    for (;;) {
        // Read the light level
        float lightLevel = analogRead(manager->_LIGHT_PIN);
        String topic = "light";
        // Check if any reads failed and exit early.
        if (isnan(lightLevel) || lightLevel < 15) {
            Serial.println(F("Failed to read from light sensor!"));
            manager->handleError("Failed to read from light sensor!");
            manager->publishData(topic.c_str(), "Offline");
        } else {
            Serial.println(lightLevel);
            manager->publishData(topic.c_str(), "Online");
        }

        // Delay between reads
        vTaskDelay(147100 / portTICK_PERIOD_MS);
    }
}

void IoTNetworkManagement::mqttLoopTask(void *parameter) {
    if (parameter == nullptr) {
        Serial.println("Error: parameter is null");
        return;
    }
    IoTNetworkManagement *instance = static_cast<IoTNetworkManagement *>(parameter);
    for (;;) {
        instance->_mqttClient.poll();
        vTaskDelay(pdMS_TO_TICKS(20)); // Delay curto para evitar uso excessivo da CPU
    }
}

void IoTNetworkManagement::errorReportTask(void *parameter) {
    if (parameter == nullptr) {
        Serial.println("Error: parameter is null");
        return;
    }
    IoTNetworkManagement* instance = static_cast<IoTNetworkManagement*>(parameter);

    for (;;) {
        instance->_preferences.begin("nvs", true);
        String lastError = instance->_preferences.getString("last_error", "");
        instance->_preferences.end();

        if (lastError.length() > 0) {
            instance->publishData("last_error", lastError.c_str());
        }

        vTaskDelay(pdMS_TO_TICKS(179051)); // Espera por 3 minutos
    }
}















