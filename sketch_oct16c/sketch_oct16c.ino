#include <TinyGPS++.h>

TinyGPSPlus gps;
unsigned long lastDebugTime = 0;
const unsigned long DEBUG_INTERVAL = 1000;  // 1 giây

// Biến đếm số câu NMEA
struct {
    unsigned long totalMessages = 0;
    unsigned long gga = 0;  // Global Positioning
    unsigned long rmc = 0;  // Recommended Minimum
    unsigned long gll = 0;  // Geographic Position
    unsigned long vtg = 0;  // Course over Ground
    unsigned long gsv = 0;  // Satellites in View
    unsigned long gsa = 0;  // DOP and Active Satellites
} nmeaCount;

void setup() {
    Serial.begin(115200);
    Serial1.begin(9600);
    
    Serial.println(F("GPS Debug Tool v2"));
    Serial.println(F("================="));
}

void loop() {
    // Đọc dữ liệu GPS
    while (Serial1.available() > 0) {
        char c = Serial1.read();
        gps.encode(c);
        
        // Đếm các loại câu NMEA
        static char buffer[10];
        static int bufIdx = 0;
        
        if (c == '$') {
            bufIdx = 0;
        } else if (bufIdx < 6) {
            buffer[bufIdx++] = c;
            if (bufIdx == 6) {
                buffer[6] = '\0';
                nmeaCount.totalMessages++;
                
                if (strcmp(buffer, "GPGGA") == 0) nmeaCount.gga++;
                else if (strcmp(buffer, "GPRMC") == 0) nmeaCount.rmc++;
                else if (strcmp(buffer, "GPGLL") == 0) nmeaCount.gll++;
                else if (strcmp(buffer, "GPVTG") == 0) nmeaCount.vtg++;
                else if (strcmp(buffer, "GPGSV") == 0) nmeaCount.gsv++;
                else if (strcmp(buffer, "GPGSA") == 0) nmeaCount.gsa++;
            }
        }
    }

    // In thông tin debug mỗi giây
    if (millis() - lastDebugTime >= DEBUG_INTERVAL) {
        lastDebugTime = millis();
        
        Serial.println(F("\n=== GPS Status ==="));
        Serial.print(F("Runtime: ")); 
        Serial.print(millis() / 1000); 
        Serial.println(F(" seconds"));
        
        // Thông tin vệ tinh
        Serial.print(F("Satellites in view: "));
        Serial.println(gps.satellites.value());
        
        Serial.print(F("HDOP: "));
        if (gps.hdop.isValid())
            Serial.println(gps.hdop.value());
        else
            Serial.println(F("No Data"));
            
        // Tình trạng fix
        Serial.print(F("Location Valid: "));
        Serial.println(gps.location.isValid() ? "Yes" : "No");
        
        if (gps.location.isValid()) {
            Serial.print(F("Latitude: "));
            Serial.println(gps.location.lat(), 6);
            Serial.print(F("Longitude: "));
            Serial.println(gps.location.lng(), 6);
            Serial.print(F("Fix Age: "));
            Serial.print(gps.location.age());
            Serial.println(F("ms"));
        }
        
        // Thống kê câu NMEA
        Serial.println(F("\n=== NMEA Statistics ==="));
        Serial.print(F("Total Messages: ")); Serial.println(nmeaCount.totalMessages);
        Serial.print(F("GGA Messages: ")); Serial.println(nmeaCount.gga);
        Serial.print(F("RMC Messages: ")); Serial.println(nmeaCount.rmc);
        Serial.print(F("GLL Messages: ")); Serial.println(nmeaCount.gll);
        Serial.print(F("VTG Messages: ")); Serial.println(nmeaCount.vtg);
        Serial.print(F("GSV Messages: ")); Serial.println(nmeaCount.gsv);
        Serial.print(F("GSA Messages: ")); Serial.println(nmeaCount.gsa);
        
        Serial.println(F("=================="));
    }
}