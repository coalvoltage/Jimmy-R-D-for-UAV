#include <SoftwareSerial.h> 
#include <TinyGPS.h> 
float lat = 28.5458,lon = 77.1703; // create variable for latitude and longitude object  
SoftwareSerial gpsSerial(8,9);//rx,tx 
TinyGPS gps; // create gps object 

void setup(){ 
  Serial.begin(9600); // be sure to check lower right corner info in an open 'serial windows'
  gpsSerial.begin(57600);  // may be 4800, 19200,38400 or 57600
} 
void loop(){ 
  while(gpsSerial.available()){ // check for gps data 
    
    if(gps.encode(gpsSerial.read()))// encode gps data 
      {
        Serial.println("awoo");
        gps.f_get_position(&lat,&lon); // get latitude and longitude 
        // display position 
        Serial.print("Position: "); 
        Serial.print("Latitude:"); 
        Serial.print(lat,6); 
        Serial.print(";"); 
        Serial.print("Longitude:"); 
        Serial.println(lon,6);   
        Serial.print(lat); 
        Serial.print(" "); 
      } 
  } 
  String latitude = String(lat,6); 
  String longitude = String(lon,6); 
  Serial.println(latitude+";"+longitude); 
  delay(1000); 
}
/*
#include <SoftwareSerial.h>

SoftwareSerial gps(3,4); // RX, TX  Connect GPS TX to pin 3
void setup()
{
  Serial.begin(115200); // be sure to check lower right corner info in an open 'serial windows'
  gps.begin(9600);  // may be 4800, 19200,38400 or 57600

}

void loop()
{
  if (gps.available())  Serial.write(gps.read());
}*/
