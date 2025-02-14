#include <TinyGPS++.h>
#include <HardwareSerial.h>

// Define GPS module connection on UART2 (GPIO16=RX2, GPIO17=TX2)
static const int RXPin = 16, TXPin = 17;
static const uint32_t GPSBaud = 9600;

TinyGPSPlus gps;
HardwareSerial gpsSerial(2);

void setup() {
    Serial.begin(115200);
    gpsSerial.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin);
    
    Serial.println("L80-R GPS Module Initialized...");
}

void loop() {
    while (gpsSerial.available() > 0) {
        char c = gpsSerial.read();
        gps.encode(c);  // Feed characters to the TinyGPS++ library

        if (gps.location.isUpdated()) {  // Check if new GPS data is available
            Serial.print("Latitude: "); 
            Serial.print(gps.location.lat(), 6);
            Serial.print(" | Longitude: "); 
            Serial.print(gps.location.lng(), 6);
            Serial.print(" | Altitude: "); 
            Serial.print(gps.altitude.meters());
            Serial.print("m | Satellites: ");
            Serial.println(gps.satellites.value());
        }
    }
}
