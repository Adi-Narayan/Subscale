#include <PID_v1.h>

#include <SPI.h>
#include "SPIFFS.h"
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <ESP32Servo.h>
#include <QuickPID.h>

//definitions

//for imu
//ulta connect karna
#define BNO_RX 16 //mpu1
#define BNO_TX 17 //mpu1
HardwareSerial BNO_UART(1); //mpu1
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

//for MS5607
#define CMD_RESET 0x1E // Reset command
#define CMD_PROM_READ 0xA0 // Base address for PROM reads
#define CMD_CONVERT_D1 0x48 // Start pressure conversion (OSR = 4096)
#define CMD_CONVERT_D2 0x58 // Start temperature conversion (OSR = 4096)
#define CMD_ADC_READ 0x00 // Read ADC result

// SPI Pins (adjust as needed)
#define CS_PIN 5  // Chip Select (CS) for MS5607

//for gps

//End of definitions

//variables
// acceleration along directions
float bno_x;
float bno_y;
float bno_z;

//pressure
uint32_t adcValue;
float pressureMS;

//file
File logfile;

//for airbrakes
Servo airbrakeServo;
const float targetApogee = 10000.0;
const int servoMinAngle = 0;
const int servoMaxAngle = 48;

double altitude, velocity;
double predictedApogee, error;
double pidOutput;
//PID parameters
double Kp = 2.0, Ki = 0.5, Kd = 1.0;
QuickPID airbrakePID(&pidOutput, &predictedApogee, &targetApogee, Kp, Ki, Kd);



//end of variables

//IMU
bool initBNO()
{
  BNO_UART.begin(115200, SERIAL_8N1, BNO_RX, BNO_TX);
  if(!bno.begin())
  {
    // Serial.println("Failed to initialise BNO055 over UART");
    return 0;
  }
  else
  {
    //bno.setExtCrystalUse(true); //external crystal jo merko nahi chahiye, but putting it out there that it's possible
    // Serial.println("Initialised BNO055 over UART");
    return 1;
  }

}

//get imu data
void getBNOData()
{
  sensors_event_t event;
  bno.getEvent(&event);
  Serial.println("Orientation X, Y, Z:");
  bno_x = event.orientation.x;
  bno_y = event.orientation.y;
  bno_z = event.orientation.z;
}

//get pressure data
//resetting MS5607
void resetMS5607() 
{
  digitalWrite(CS_PIN, LOW);       // Select the sensor
  SPI.transfer(CMD_RESET);        // Send reset command
  digitalWrite(CS_PIN, HIGH);     // Deselect the sensor
  delay(3);                       // Wait for reset to complete
}

// Function to read ADC value for pressure
void readADC(uint8_t cmd) {
  // Start conversion
  digitalWrite(CS_PIN, LOW);       // Select the sensor
  SPI.transfer(cmd);               // Send conversion command
  digitalWrite(CS_PIN, HIGH);      // Deselect the sensor
  delay(10);                       // Wait for conversion (10ms for OSR 4096)

  // Read ADC result
  digitalWrite(CS_PIN, LOW);       // Select the sensor
  SPI.transfer(CMD_ADC_READ);      // Send ADC read command

  // Read 24-bit ADC value (MSB first)
  value = SPI.transfer(0x00);
  value = (value << 8) | SPI.transfer(0x00);
  value = (value << 8) | SPI.transfer(0x00);

  digitalWrite(CS_PIN, HIGH);      // Deselect the sensor

  return value;
}
//end of ms607

//bc gps ka kaun lega readings?

//flash data logging
void logData()
{
  //
  File logfile = SPIFFS.open("/datalogs.txt", FILE_WRITE);
  if (logfile) 
  {
    logfile.println(parameter);
    //call all get data functions
    logfile.close();
  } 
  else 
  {
    Serial.println("Could not open file.");
  }
}

//send data by rylr


//kalman filter the pressure data
//sensor fusion
// void kalmanFilter(float measured_altitude, float accel_y) {
//     // Step 1: State Prediction
//     float accel_earth = accel_y + g; // Convert to earth frame
//     x[0] += x[1] * dt + 0.5 * accel_earth * dt * dt; // Altitude update
//     x[1] += accel_earth * dt; // Velocity update
    
//     // Predict covariance matrix
//     P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q[0][0]);
//     P[0][1] += dt * (P[1][1] + Q[0][1]);
//     P[1][0] += dt * (P[1][1] + Q[1][0]);
//     P[1][1] += Q[1][1];
    
//     // Step 2: Measurement Update
//     float y = measured_altitude - x[0]; // Innovation
//     float S = P[0][0] + R;
//     float K[2] = {P[0][0] / S, P[1][0] / S}; // Kalman Gain
    
//     // Update estimates
//     x[0] += K[0] * y;
//     x[1] += K[1] * y;
    
//     // Update covariance matrix
//     P[0][0] -= K[0] * P[0][0];
//     P[0][1] -= K[0] * P[0][1];
//     P[1][0] -= K[1] * P[0][0];
//     P[1][1] -= K[1] * P[0][1];
// }

//servos for airbrakes
void initAirbrakes()
{
  airbrakeServo.attach(9);
  airbrakeServo.write(servoMinAngle);

  airbrakePID.SetMode(AUTOMATIC);
  airbrakePID.SetOutputLimits(0,100);
}

// void airbrakes()
// {

//     // Predict apogee using RK4 Method (simplified placeholder calculation)
//     predictedApogee = altitude + (velocity * 5.0); // Simplified estimation
    
//     // Compute error
//     error = targetApogee - predictedApogee;
    
//     // Apply PID control
//     airbrakePID.Compute();
//     int servoAngle = map(pidOutput, 0, 100, servoMinAngle, servoMaxAngle);
    
//     // Fail-safe check for excessive tilt
//     if (abs(getRocketTilt()) >= 10) {
//         airbrakeServo.write(servoMinAngle); // Retract airbrakes fully
//         Serial.println("Fail-safe triggered: Excessive tilt detected!");
//         return;
//     }
//      // Check if apogee is reached
//     if (altitude >= targetApogee) {
//         airbrakeServo.write(servoMinAngle); // Fully retract airbrakes
//         Serial.println("Apogee reached. Airbrakes retracted.");
//         while (1); // Stop further execution
//     }
    
//     delay(100);
// }



//apogee detection


void setup() 
{
  Serial.begin(115200);

  // For BNO055
  if(!initBNO())
    Serial.println("Failed to initialise BNO055 over UART");
  else
    Serial.println("Initialised BNO055 over UART");

  // For MS5607
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH); // Deselect MS5607 
  resetMS5607(); // Reset MS5607
  Serial.println("MS5607 initialized.");

  // for SPIFFS
  if(!SPIFFS.begin())//or begin(true), needs to be tested on module tho
  {
    Serial.println("SPI Flash Init Failed.");
  }
  else//need to complete
  { //on a 16MB flash size wala module, we get around 11MB of storage
    // File logfile = SPIFFS.open("/datalogs.txt", FILE_WRITE);
    logfile.println("Flash initialised. ");
  }

}


void loop() 
{
  // put your main code here, to run repeatedly:

}
