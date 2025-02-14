#include <SPI.h>

#define CMD_RESET 0x1E // Reset command
#define CMD_PROM_READ 0xA0 // Base address for PROM reads
#define CMD_CONVERT_D1 0x48 // Start pressure conversion (OSR = 4096)
#define CMD_CONVERT_D2 0x58 // Start temperature conversion (OSR = 4096)
#define CMD_ADC_READ 0x00 // Read ADC result


#define CS_PIN 5  

void setup() {
  Serial.begin(115200);
  

  SPI.begin();
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH); 
  
  // Reset MS5607
  resetMS5607();
  Serial.println("MS5607 initialized.");
}

void loop() {
  // Read raw pressure and temperature
  uint32_t rawPressure = readADC(CMD_CONVERT_D1);
  uint32_t rawTemperature = readADC(CMD_CONVERT_D2);

  // Print raw values (you'll need to convert these using calibration coefficients)
  Serial.print("Raw Pressure: ");
  Serial.println(rawPressure);
  Serial.print("Raw Temperature: ");
  Serial.println(rawTemperature);

  delay(1000);
}

// Function to reset MS5607
void resetMS5607() {
  digitalWrite(CS_PIN, LOW);       // Select the sensor
  SPI.transfer(CMD_RESET);        // Send reset command
  digitalWrite(CS_PIN, HIGH);     // Deselect the sensor
  delay(3);                       // Wait for reset to complete
}

// Function to read ADC value
uint32_t readADC(uint8_t cmd) {
  // Start conversion
  digitalWrite(CS_PIN, LOW);       // Select the sensor
  SPI.transfer(cmd);               // Send conversion command
  digitalWrite(CS_PIN, HIGH);      // Deselect the sensor
  delay(10);                       // Wait for conversion (10ms for OSR 4096)

  // Read ADC result
  digitalWrite(CS_PIN, LOW);       // Select the sensor
  SPI.transfer(CMD_ADC_READ);      // Send ADC read command

  // Read 24-bit ADC value (MSB first)
  uint32_t value = SPI.transfer(0x00);
  value = (value << 8) | SPI.transfer(0x00);
  value = (value << 8) | SPI.transfer(0x00);

  digitalWrite(CS_PIN, HIGH);      // Deselect the sensor

  return value;
}
