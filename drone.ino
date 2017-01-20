// Libraries
#include "Adafruit_FONA.h"
#include <SoftwareSerial.h>
//#include "DHT.h"
#include <ArduinoJson.h>
#include "math.h"
// Pins
#define FONA_RX 2
#define FONA_TX 10
#define FONA_RST 4

// Buffer
char replybuffer[255];

// Instances
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;

// Fona instance
Adafruit_FONA fona = Adafruit_FONA(FONA_RST);
uint8_t type;



void setup() {

  setUpFona(); //used to set up fona 

}



void loop() {

  float distance,bearing,origin_lon,origin_lat,dest_lat,dest_lon;

  
  get_drone_location(origin_lat,origin_lon); // gets drone location
  
  
  get_destination(dest_lat,dest_lon); //get destination from server

  
  distance = calc_distance(origin_lat,origin_lon,dest_lat,dest_lon); //calculate distance to target

  
  bearing = calc_bearing(origin_lat,origin_lon,dest_lat,dest_lon); //calculate heading to target

  
  
  
  // Wait
  delay(60 * 1000);
  
}

void setUpFona()
{
  // Initi serial
  while (!Serial);
  Serial.begin(9600);
  Serial.println(F("FONA reading SMS"));
  Serial.println(F("Initializing....(May take 3 seconds)"));

  fonaSerial->begin(9600);
  if (! fona.begin(*fonaSerial)) {
    Serial.println(F("Couldn't find FONA"));
    while (1);
  }
  type = fona.type();
  delay(4000);

  Serial.println("enabling GPS");
  while(!fona.enableGPS(true));
  Serial.println("GPS enabled");

  fona.setGPRSNetworkSettings(F("ppinternet"));
  Serial.println("delay start");
  delay(10000);
  Serial.println("delay stop");

  Serial.println("enabling GPRS");
  while(!fona.enableGPRS(true));
  Serial.println("GPRS enabled");
}



float calc_distance(float lat1,float lon1, float lat2, float lon2)
{

  float R = 6371e3;
  
  //Haversine Distance
  float lat1_rad = deg_to_rad(lat1);
  float lat2_rad = deg_to_rad(lat2);
  float delta_lat_rad = deg_to_rad(lat2-lat1);
  float delta_lon_rad = deg_to_rad(lon2-lon1);

  float a = sin(delta_lat_rad/2) * sin(delta_lat_rad/2) +
            cos(lat1_rad) * cos(lat2_rad) *
            sin(delta_lon_rad/2) * sin(delta_lon_rad/2);

  float c = 2 * atan(sqrt(a)/sqrt(1-a));

  float d = R * c;

  return d;

}

//returns bearing in degrees
float calc_bearing(float lat1,float lon1,float lat2, float lon2)
{
  float y = sin(lon2-lon1) * cos(lat2);
  float x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(lon2-lon1);
  float brng = rad_to_deg(atan(y/x));
  
  
  return brng;
}


//converts degrees to radians
float deg_to_rad(float degrees)
{
  return (degrees* M_PI)/180;
}

//converts radians to degrees
float rad_to_deg(float radians)
{
  return (radians*180)/M_PI;
}


void apply_pitch(float value)
{
  // call MSP API to apply pitch to drone
}

void apply_yaw(float value)
{
  // call MSP API to apply yaw to drone
}

void apply_roll(float value)
{
  // call MSP API to apply roll to drone
}

void apply_throttle(float throttle)
{
  // call MSP API to apply throttle to drone
}



bool get_drone_location(float& lat, float& lon)
{
  float latitude, longitude, speed_kph, heading, altitude;
  bool gpsFix = fona.getGPS(&latitude, &longitude, &speed_kph, &heading, &altitude);


  lat = convertDegMinToDecDeg(latitude);
  lon = convertDegMinToDecDeg(longitude);

  Serial.print(lat);
  Serial.print(lon);

  return gpsFix;
  
}

//used to convert fona gps output to actual location coordinate in degrees
float convertDegMinToDecDeg (float degMin) {
  double min = 0.0;
  double decDeg = 0.0;
 
  //get the minutes, fmod() requires double
  min = fmod((double)degMin, 100.0);
 
  //rebuild coordinates in decimal degrees
  degMin = (int) ( degMin / 100 );
  decDeg = degMin + ( min / 60 );
 
  return (float) decDeg;
}


//gets gps coordinates from server
bool get_destination(float& lat, float &lon)
{
   // Prepare request
  uint16_t statuscode;
  int16_t length;
  String domain = "ADD_DOMAIN_HERE";
  String url = domain + "/location/";
  char buf[80];
  url.toCharArray(buf, 100);

  Serial.print("Request: ");
  Serial.print(buf);
  
  // Get location
  if (!fona.HTTP_GET_start(buf, &statuscode, (uint16_t *)&length)) {
    Serial.println("Failed!");
    return false;
  }

  char response[255];
  StaticJsonBuffer<200> jsonBuffer;
  int i=0;
  int tmplength=length;
  
  while (length > 0) {
    
    while (fona.available()) {
      char c = fona.read();
           
      // Serial.write is too slow, we'll write directly to Serial register!
      #if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
        loop_until_bit_is_set(UCSR0A, UDRE0); /* Wait until data register empty. */
        UDR0 = c;
      #else
        response[i] = c;
        Serial.write(c);
      #endif
      length--;
      i++;
    }
  }
  fona.HTTP_GET_end();

  if(tmplength>0)
  {
    response[tmplength] = '\0';
    JsonObject& root = jsonBuffer.parseObject(response);

    if (root.success())
    { 
      lat = root["lat"].as<float>();
      lon = root["lon"].as<float>();

      return true;
      
    }
  }

  return false;
}

