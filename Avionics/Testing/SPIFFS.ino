#include <SPIFFS.h>

void setup() 
{
  Serial.begin(115200);

  if(!SPIFFS.begin())//or begin(true), needs to be tested on module tho
  {
    Serial.println("SPIFFS Mount Failed.");
    return;
  }

  File logfile = SPIFFS.open("/datalogs.txt", FILE_WRITE);
  if(logfile)
  {
    logfile.println("Writing to txt file on internal SPI Flash. pls work");
    logfile.close();
    Serial.println("Written to file.");
  }
  else
    Serial.println("Failed to open file.");
}

void loop() {
  // File logfile = SPIFFS.open("/datalogs.txt", FILE_READ);
  // if(logfile)
  // {
    
  // }
  // //whatever

}
