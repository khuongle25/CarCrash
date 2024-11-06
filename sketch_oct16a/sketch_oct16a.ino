#include <Arduino_LSM6DSOX.h>
#include <TinyGPS++.h>
#include <WiFiNINA.h>
#include <car-crash_inferencing.h>

// WiFi settings
const char ssid[] = "Khuong";
const char pass[] = "Khuongle25";

// Web server
WiFiServer server(80);

// GPS parser
TinyGPSPlus gps;

// Sensor data struct
struct SensorData {
    float accel[3] = {0, 0, 0};  // x, y, z
    float lat = 0, lng = 0;
    float alt = 0, speed = 0;
    int sats = 0;
    bool gpsValid = false;
    unsigned long lastGPSUpdate = 0;
} sensorData;

// Buffer for acceleration data
static float features[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];
size_t feature_index = 0;

// Crash detection variables
bool crash_detected = false;
unsigned long last_crash_time = 0;
const unsigned long CRASH_COOLDOWN = 500; // 10 seconds cooldown between crash detections

void setup() {
    Serial.begin(115200);
    Serial1.begin(9600);  // GPS
    
    // Initialize IMU
    if (!IMU.begin()) {
        Serial.println("Failed to initialize IMU!");
        while (1);
    }

    // Connect to WiFi
    WiFi.begin(ssid, pass);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    
    server.begin();
}

// Function to get feature data for inference
int raw_feature_get_data(size_t offset, size_t length, float *out_ptr) {
    memcpy(out_ptr, features + offset, length * sizeof(float));
    return 0;
}

void loop() {
    // Read and process sensor data
    readSensors();
    
    // Process crash detection if we have enough data
    if (feature_index == EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
        process_crash_detection();
        // Reset feature index for next batch
        feature_index = 0;
    }
    
    // Handle web clients
    handleWebClients();
}

void readSensors() {
    // Read IMU
    if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(sensorData.accel[0], sensorData.accel[1], sensorData.accel[2]);
        
        // Add to features buffer if space available
        if (feature_index < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
            features[feature_index++] = sensorData.accel[0];
            if (feature_index < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
                features[feature_index++] = sensorData.accel[1];
            }
            if (feature_index < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
                features[feature_index++] = sensorData.accel[2];
            }
        }
    }

    // Read GPS
    while (Serial1.available() > 0) {
        if (gps.encode(Serial1.read())) {
            updateGPSData();
        }
    }
}
// [Các include và khai báo biến giữ nguyên...]

void process_crash_detection() {
    // Skip if in cooldown period
    if (millis() - last_crash_time < CRASH_COOLDOWN) {
        return;
    }

    ei_impulse_result_t result = { 0 };

    // Create signal from features buffer
    signal_t features_signal;
    features_signal.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
    features_signal.get_data = &raw_feature_get_data;

    // Run the classifier
    EI_IMPULSE_ERROR res = run_classifier(&features_signal, &result, false);
    
    if (res != EI_IMPULSE_OK) {
        Serial.print("ERR: Failed to run classifier (");
        Serial.print(res);
        Serial.println(")");
        return;
    }

    // Check for crash detection
    for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        if (strcmp(ei_classifier_inferencing_categories[i], "crash") == 0) {
            if (result.classification[i].value > 0.8) { // Threshold at 80% confidence
                crash_detected = true;
                last_crash_time = millis();
                handle_crash_event();
            } else {
                crash_detected = false;
                handle_crash_event();
            }
        }
    }
}

void print_inference_result(ei_impulse_result_t result) {
    // Print timing info
    Serial.print("Timing: DSP ");
    Serial.print(result.timing.dsp);
    Serial.print(" ms, inference ");
    Serial.print(result.timing.classification);
    Serial.print(" ms, anomaly ");
    Serial.print(result.timing.anomaly);
    Serial.println(" ms");

    // Print predictions
    Serial.println("Predictions:");
    for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        Serial.print("  ");
        Serial.print(ei_classifier_inferencing_categories[i]);
        Serial.print(": ");
        Serial.println(result.classification[i].value, 5);
    }
}

void handle_crash_event() {
    Serial.println("CRASH DETECTED!");
    
    // Print crash location and details
    Serial.println("Crash Details:");
    Serial.print("Location: ");
    if (sensorData.gpsValid) {
        Serial.print("Lat: "); 
        Serial.print(sensorData.lat, 6);
        Serial.print(" Lng: "); 
        Serial.println(sensorData.lng, 6);
        Serial.print("Speed: "); 
        Serial.print(sensorData.speed);
        Serial.println(" km/h");
    } else {
        Serial.println("GPS location not available");
    }
    
    Serial.print("Acceleration at impact - X: ");
    Serial.print(sensorData.accel[0]);
    Serial.print(" Y: ");
    Serial.print(sensorData.accel[1]);
    Serial.print(" Z: ");
    Serial.println(sensorData.accel[2]);
}

// [Các hàm khác giữ nguyên...]
void updateGPSData() {
    if (gps.location.isValid()) {
        sensorData.gpsValid = true;
        sensorData.lat = gps.location.lat();
        sensorData.lng = gps.location.lng();
        sensorData.lastGPSUpdate = millis();
        
        if (gps.altitude.isValid()) {
            sensorData.alt = gps.altitude.meters();
        }
        if (gps.speed.isValid()) {
            sensorData.speed = gps.speed.kmph();
        }
        if (gps.satellites.isValid()) {
            sensorData.sats = gps.satellites.value();
        }
    }
}

void handleWebClients() {
    WiFiClient client = server.available();
    if (client) {
        String currentLine = "";
        while (client.connected()) {
            if (client.available()) {
                char c = client.read();
                if (c == '\n') {
                    if (currentLine.length() == 0) {
                        sendWebPage(client);
                        break;
                    } else {
                        currentLine = "";
                    }
                } else if (c != '\r') {
                    currentLine += c;
                }
            }
        }
        client.stop();
    }
}

void sendWebPage(WiFiClient& client) {
    // client.println(F("HTTP/1.1 200 OK"));
    // client.println(F("Content-type:text/html"));
    // client.println();
    
    // client.println(F("<!DOCTYPE html><html>"));
    // client.println(F("<head>"));
    // client.println(F("<meta charset='utf-8'>"));
    // client.println(F("<meta name='viewport' content='width=device-width, initial-scale=1'>"));
    // client.println(F("<title>Car Crash Monitor</title>"));
    // client.println(F("<meta http-equiv='refresh' content='1'>")); 
    
    // client.println(F("<style>"));
    // client.println(F("body{font-family:Arial;margin:20px;background:#f0f0f0}"));
    // client.println(F(".container{max-width:800px;margin:0 auto;background:white;padding:20px;border-radius:10px;box-shadow:0 2px 5px rgba(0,0,0,0.1)}"));
    // client.println(F(".sensor-box{background:#f8f9fa;padding:15px;margin:10px 0;border-radius:5px;border-left:4px solid #007bff}"));
    // client.println(F(".crash-alert{background:#ffe0e0;border-left-color:#ff0000}"));
    // client.println(F(".value{font-size:1.2em;margin:5px 0}"));
    // client.println(F("</style></head>"));
    
    // client.println(F("<body><div class='container'>"));
    // client.println(F("<div class='sensor-box crash-alert'>"));
    // client.print(F("<h2>⚠️ CRASH DETECTED: "));
    // client.print(crash_detected);
    // client.print(F("⚠️</h2></div>"));
    //   // client.print(F("<div class='value'>Time since crash: "));
    //   // client.print((millis() - last_crash_time) / 1000);
    //   // client.println(F(" seconds ago</div></div>"));
    
    // // Crash status

    // // if (crash_detected && (millis() - last_crash_time < CRASH_COOLDOWN)) {
    // //     client.println(F("<div class='sensor-box crash-alert'>"));
    // //     client.println(F("<h2>⚠️ CRASH DETECTED! ⚠️</h2>"));
    // //     client.print(F("<div class='value'>Time since crash: "));
    // //     client.print((millis() - last_crash_time) / 1000);
    // //     client.println(F(" seconds ago</div></div>"));
    // // }
    
    // // Accelerometer data
    // client.println(F("<div class='sensor-box'>"));
    // client.println(F("<h2>Accelerometer</h2>"));
    // for (int i = 0; i < 3; i++) {
    //     client.print(F("<div class='value'>"));
    //     client.print(char('X' + i));
    //     client.print(F(": "));
    //     client.print(sensorData.accel[i]);
    //     client.println(F(" g</div>"));
    // }
    // client.println(F("</div>"));
    
    // // GPS data
    // client.println(F("<div class='sensor-box'>"));
    // client.println(F("<h2>GPS Data</h2>"));
    // if (sensorData.gpsValid) {
    //     client.print(F("<div class='value'>Latitude: "));
    //     client.print(sensorData.lat, 6);
    //     client.println(F("°</div>"));
    //     client.print(F("<div class='value'>Longitude: "));
    //     client.print(sensorData.lng, 6);
    //     client.println(F("°</div>"));
    //     client.print(F("<div class='value'>Altitude: "));
    //     client.print(sensorData.alt);
    //     client.println(F(" m</div>"));
    //     client.print(F("<div class='value'>Speed: "));
    //     client.print(sensorData.speed);
    //     client.println(F(" km/h</div>"));
    //     client.print(F("<div class='value'>Satellites: "));
    //     client.print(sensorData.sats);
    //     client.println(F("</div>"));
    // } else {
    //     client.println(F("<div class='value'>Waiting for GPS fix...</div>"));
    // }
    // client.println(F("</div>"));
    
    // client.println(F("</div></body></html>"));

    
}