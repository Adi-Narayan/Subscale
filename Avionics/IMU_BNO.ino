#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

//ulta connect karna
#define BNO_RX 16
#define BNO_TX 17

HardwareSerial BNO_UART(1);


Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &BNO_UART);
void setup() 
{
  Serial.begin(115200);
  BNO_UART.begin(115200, SERIAL_8N1, BNO_RX, BNO_TX);


  if(!bno.begin())
    Serial.println("Failed to initialise BNO055 over UART");
  else
  {
    Serial.println("Initialised BNO055 over UART");
  }
}

void loop() 
{
  sensors_event_t event;
  bno.getEvent(&event);

  Serial.println("Orientation X, Y, Z:");
  Serial.print(event.orientation.x);
  Serial.print(", ");
  Serial.print(event.orientation.y);
  Serial.print(", ");
  Serial.println(event.orientation.z);
  Serial.println(" ");
}
