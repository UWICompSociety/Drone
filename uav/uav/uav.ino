#include <math.h>
#include <stdint.h>

#include <Servo.h>
#include <SoftwareSerial.h>

/* #include "DHT.h" */
#include "Adafruit_FONA.h"
#include "ArduinoJson.h"

/* Pins */
#define FONA_RX         9
#define FONA_TX         8
#define FONA_RST        12
#define PITCH_PIN       3
#define YAW_PIN         5
#define ROLL_PIN        2
#define THROTTLE_PIN    4
#define AUX1_PIN        6
#define AUX2_PIN        7

#define URL             "http://stonegod21.pythonanywhere.com/location/"

static uint8_t type;
static char replybuffer[255];


byte last_channel_1;
byte last_channel_2;
byte last_channel_3;
byte last_channel_4;
byte last_channel_5;
byte last_channel_6;

unsigned long timer_1;
unsigned long timer_2;
unsigned long timer_3;
unsigned long timer_4;
unsigned long timer_5;
unsigned long timer_6;
unsigned long current_time;
int prev_time;

bool armed = false;
bool manual = false;

int current_throttle=1000;
int current_pitch=1500;
int current_roll=1500;
int current_yaw=1500;

bool goforward=false;
int forward_timer = 0;
bool stop_now=false;

int throttle_chan;
int yaw_chan;
int roll_chan;
int pitch_chan;
int aux1_chan;
int aux2_chan;

float target_distance;
float target_bearing;
float dest_lat;
float dest_lon;
float drone_lat;
float drone_lon;
bool gpsFix;

static SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
static SoftwareSerial *fonaSerial = &fonaSS;

static Servo pitchServo;
static Servo yawServo;
static Servo rollServo;
static Servo throttleServo;
static Servo aux1Servo;
static Servo aux2Servo;

static Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

static void set_up_servos(void);
static void setUpFona(void);
static float calc_distance(float lat1, float lon1, float lat2, float lon2);
static float calc_bearing(float lat1, float lon1, float lat2, float lon2);
static float deg_to_rad(float deg);
static float rad_to_deg(float rad);
static void arm(void);
static void disarm(void);
static void apply_pitch(int value);
static void apply_yaw(int value);
static void apply_roll(int value);
static void apply_throttle(int value);
static void apply_aux1(int value);
static void apply_aux2(int value);
static void check_height(void);
static void get_drone_location(float *lat, float *lon);
static float convertDegMinToDecDeg(float degMin);
static int8_t get_destination(float *lat, float *lon);
static float calc_distance_and_bearing(float,float,float,float,float*,float*);
static void  print_reciever_values();

void
setup(void)
{
    setUpFona();
    set_up_servos();
    Serial.begin(9600);
    // put your setup code here, to run once:

    //used to set up interrupt for remote reciever
    PCICR |= (1 << PCIE0);                                                    //Set PCIE0 to enable PCMSK0 scan.
    PCMSK0 |= (1 << PCINT0);                                                  //Set PCINT0 (digital input 53) to trigger an interrupt on state change.
    PCMSK0 |= (1 << PCINT1);                                                  //Set PCINT1 (digital input 52)to trigger an interrupt on state change.
    PCMSK0 |= (1 << PCINT2);                                                  //Set PCINT2 (digital input 51)to trigger an interrupt on state change.
    PCMSK0 |= (1 << PCINT3);                                                  //Set PCINT3 (digital input 50)to trigger an interrupt on state change.
    PCMSK0 |= (1 << PCINT4);                                                  //Set PCINT4 (digital input 10)to trigger an interrupt on state change.
    PCMSK0 |= (1 << PCINT5);                                                  //Set PCINT6 (digital input 11)to trigger an interrupt on state change.
}

void
loop(void)
{
    Serial.print("Getting a fix");
    delay(5000);
    
    get_drone_location(&drone_lat, &drone_lon);
    get_destination(&dest_lat, &dest_lon);
   
    dest_lat=18.005872;
    dest_lon=-76.747358;
    
    calc_distance_and_bearing(drone_lat, drone_lon, dest_lat, dest_lon,&target_distance,&target_bearing); /* distance to target */

    print_reciever_values();
    
    if(!armed && aux2_chan>=1200) //if drone is not armed and aux2 is set to 2000
    {
        apply_aux2(1000);
        apply_aux1(1000);
        apply_throttle(885); //throttle needs to be at this value for drone to be armed
        arm(); //arm the drone
    }else{
        
        if(manual)  //if drone is in the manual
        {
          apply_throttle(throttle_chan);
          apply_yaw(yaw_chan);
          apply_roll(roll_chan);
          apply_pitch(pitch_chan);
        }else{
            apply_throttle(current_throttle);
            apply_pitch(current_pitch);
            apply_yaw(current_yaw);
            apply_roll(current_roll);
            
            if(millis()-prev_time>=500 && !goforward) //throttle is increased every 500 milliseconds
            {
              prev_time=millis();
              current_throttle+=200;
              if(current_throttle>=1600)
              {
                forward_timer = millis();
                current_throttle = 1600;
                goforward = true;   //stop increasing throttle and start going forward
                prev_time=0;
                
              }
            }
            
            if(goforward && !stop_now)
            {
              current_pitch=1550; //set pitch to go forward
              if(millis()-forward_timer>=3000) //go forward for 3 seconds
              {
                current_pitch=1500;
                stop_now = true; //stop going forward and initiate land sequence
              }
            }
            
            
            if(stop_now)
            {
              if(millis()-prev_time>=1000) //every second decrease throttle
              {
                prev_time=millis();
                current_throttle-=100;
                if(current_throttle<=1000)
                {
                  current_throttle=1000;
                  stop_now=false;
                }
              }
            }
         }
        
      }
    
      if(armed && aux2_chan <= 1200)
      {
        disarm(); //disarm if aux2 is set to 1000
      }
    
      
      if(aux1_chan<=1200) //used to set to manual control
      {
        manual = true;
      }else if(aux1_chan>=1700){ //used to set to automatic
        manual = false;
      }
    
      if(aux1_chan>=1490 && aux1_chan<=1520) //used for gtune
      {
        apply_aux1(aux1_chan);
      }else{
        apply_aux1(aux1_chan);
      }

   
}

static void
set_up_servos(void)
{
    pinMode(PITCH_PIN, OUTPUT);
    pitchServo.attach(PITCH_PIN);
    
    pinMode(ROLL_PIN, OUTPUT);
    rollServo.attach(ROLL_PIN);
    
    pinMode(YAW_PIN, OUTPUT);
    yawServo.attach(YAW_PIN);
    
    pinMode(THROTTLE_PIN, OUTPUT);
    throttleServo.attach(THROTTLE_PIN);
    
    pinMode(AUX1_PIN, OUTPUT);
    aux1Servo.attach(AUX1_PIN);
    
    pinMode(AUX2_PIN, OUTPUT);
    aux2Servo.attach(AUX2_PIN);
}

static void
setUpFona(void)
{
    while (!Serial);
    Serial.begin(9600);

    Serial.println(F("FONA reading SMS"));
    Serial.println(F("Initializing....(May take 3 seconds)"));
    
    fonaSerial->begin(9600);
    if (!fona.begin(*fonaSerial)) {
        Serial.println(F("Couldn't find FONA"));
        while (1);
    }
    type = fona.type();
    delay(4000);
    
    Serial.println("enabling GPS");
    while (!fona.enableGPS(true));
    Serial.println("GPS enabled");
    
    fona.setGPRSNetworkSettings(F("ppinternet"));
    Serial.println("delay start");
    delay(10000);
    Serial.println("delay stop");
    
    Serial.println("enabling GPRS");
    while (!fona.enableGPRS(true));
    Serial.println("GPRS enabled");

    
}

static float
calc_distance_and_bearing(float lat1, float lon1, float lat2, float lon2,float* distance,float*bearing)
{
    float R = 6371e3;
    
    //Haversine Distance
    float lat1_rad = deg_to_rad(lat1);
    float lat2_rad = deg_to_rad(lat2);
    float delta_lat_rad = deg_to_rad(lat2 - lat1);
    float delta_lon_rad = deg_to_rad(lon2 - lon1);
    
    float a = sin(delta_lat_rad / 2) * sin(delta_lat_rad / 2)
        + cos(lat1_rad) * cos(lat2_rad)
        * sin(delta_lon_rad / 2) * sin(delta_lon_rad / 2);
    
    float c = 2 * atan(sqrt(a) / sqrt(1 - a));
    
    *distance = R * c;

    float y = sin(lon2 - lon1) * cos(lat2);
    float x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(lon2 - lon1);
    *bearing = rad_to_deg(atan(y / x));


}

//converts degrees to radians
static float
deg_to_rad(float deg)
{
    return (deg * M_PI) / 180;
}

//converts radians to degrees
static float
rad_to_deg(float rad)
{
    return (rad * 180) / M_PI;
}



static void
apply_pitch(int value)
{
    pitchServo.write(value);
    // apply PWM for pitch
}

static void
apply_yaw(int value)
{
    yawServo.write(value);
    // apply PWM for yaw
}

static void
apply_roll(int value)
{
    rollServo.write(value);
    // call MSP API to apply roll to drone
}

static void
apply_throttle(int value)
{
    throttleServo.write(value);
    // call MSP API to apply throttle to drone
}

static void
apply_aux1(int value)
{
    aux1Servo.write(value);
}

static void
apply_aux2(int value)
{
    aux2Servo.write(value);
}

static void
check_height(void)
{
    /* TODO */ 
}

static void arm(void)
{
  apply_aux2(1800);//moves from out of range to into range
  apply_aux1(1400);
  armed = true;
}
static void disarm(void)
{
  apply_aux2(1200);
  current_throttle = 1000;
  armed = false;
}

static void
get_drone_location(float *lat, float *lon)
{
    float latitude, longitude, speed_kph, heading, altitude;
    gpsFix = fona.getGPS(&latitude, &longitude, &speed_kph, &heading, &altitude);
    
    *lat = latitude;
    *lon = longitude;

//    Serial.print("Fix ");
//    Serial.println(gpsFix);
//    Serial.println(latitude);
//    Serial.println(longitude);
//    Serial.println(heading);
//    Serial.println(altitude);
//    Serial.println(speed_kph);
   
}

static void 
print_reciever_values()
{
  Serial.print("Throttle:");
  Serial.print(throttle_chan);

  Serial.print("Yaw:");
  Serial.print(yaw_chan);

  Serial.print("Roll:");
  Serial.print(roll_chan);

  Serial.print("Pitch:");
  Serial.print(pitch_chan);

  Serial.print("Aux1:");
  Serial.print(aux1_chan);

  Serial.print("Aux2:");
  Serial.println(aux2_chan);

  
}

//used to convert fona gps output to actual location coordinate in degrees
static float
convertDegMinToDecDeg(float degMin)
{
    double min = 0.0;
    double decDeg = 0.0;
    
    //get the minutes, fmod() requires double
    min = fmod((double) degMin, 100.0);
    
    //rebuild coordinates in decimal degrees
    degMin = (int) ( degMin / 100 );
    decDeg = degMin + ( min / 60 );
    
    return (float) decDeg;
}

//gets gps coordinates from server
static int8_t
get_destination(float *lat, float *lon)
{
    int i;
    int tmplength;
    char c;
    uint16_t len;
    uint16_t statuscode;
    char response[256];
    StaticJsonBuffer<200> jsonBuffer;
    
    Serial.print("Request: ");
    Serial.print(URL);
    
    // Get location
    if (!fona.HTTP_GET_start(URL, &statuscode, &len)) {
        Serial.println("Failed!");
        return -1;
    }

    if (statuscode != 200)
        return -1;
    
    i = 0;
    tmplength = len;
    
    while (len > 0) {  
        while (fona.available() > 0) {
            c = fona.read();
            if (c == -1)
                continue;
            
            // Serial.write is too slow, we'll write directly to Serial register!
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
            loop_until_bit_is_set(UCSR0A, UDRE0); /* Wait until data register empty. */
            UDR0 = c;
#else
            response[i] = c;
            Serial.write(c);
#endif
            len--;
            i++;
        }
    }
    
    fona.HTTP_GET_end();

    if (tmplength > 0) {
        response[tmplength] = '\0';
        JsonObject& root = jsonBuffer.parseObject(response);
    
        if (root.success()) { 
            *lat = root["lat"].as<float>();
            *lon = root["lon"].as<float>();
            
            return 0;
        }
    }

    return -1;
}

ISR(PCINT0_vect){
  current_time = micros();
  //Channel 1=========================================
  if(PINB & B00000001){                                                     //Is input 8 high?
    if(last_channel_1 == 0){                                                //Input 8 changed from 0 to 1.
      last_channel_1 = 1;                                                   //Remember current input state.
      timer_1 = current_time;                                               //Set timer_1 to current_time.
    }
  }
  else if(last_channel_1 == 1){                                             //Input 8 is not high and changed from 1 to 0.
    last_channel_1 = 0;                                                     //Remember current input state.
    roll_chan = current_time - timer_1;                             //Channel 1 is current_time - timer_1.
  }
  //Channel 2=========================================
  if(PINB & B00000010 ){                                                    //Is input 9 high?
    if(last_channel_2 == 0){                                                //Input 9 changed from 0 to 1.
      last_channel_2 = 1;                                                   //Remember current input state.
      timer_2 = current_time;                                               //Set timer_2 to current_time.
    }
  }
  else if(last_channel_2 == 1){                                             //Input 9 is not high and changed from 1 to 0.
    last_channel_2 = 0;                                                     //Remember current input state.
    throttle_chan = current_time - timer_2;                             //Channel 2 is current_time - timer_2.
  }
  //Channel 3=========================================
  if(PINB & B00000100 ){                                                    //Is input 10 high?
    if(last_channel_3 == 0){                                                //Input 10 changed from 0 to 1.
      last_channel_3 = 1;                                                   //Remember current input state.
      timer_3 = current_time;                                               //Set timer_3 to current_time.
    }
  }
  else if(last_channel_3 == 1){                                             //Input 10 is not high and changed from 1 to 0.
    last_channel_3 = 0;                                                     //Remember current input state.
    pitch_chan = current_time - timer_3;                             //Channel 3 is current_time - timer_3.

  }
  //Channel 4=========================================
  if(PINB & B00001000 ){                                                    //Is input 11 high?
    if(last_channel_4 == 0){                                                //Input 11 changed from 0 to 1.
      last_channel_4 = 1;                                                   //Remember current input state.
      timer_4 = current_time;                                               //Set timer_4 to current_time.
    }
  }
  else if(last_channel_4 == 1){                                             //Input 11 is not high and changed from 1 to 0.
    last_channel_4 = 0;                                                     //Remember current input state.
    yaw_chan = current_time - timer_4;                             //Channel 4 is current_time - timer_4.
  }

   //Channel 5=========================================
  if(PINB & B00010000 ){                                                    //Is input 11 high?
    if(last_channel_5 == 0){                                                //Input 11 changed from 0 to 1.
      last_channel_5 = 1;                                                   //Remember current input state.
      timer_5 = current_time;                                               //Set timer_4 to current_time.
    }
  }
  else if(last_channel_5 == 1){                                             //Input 11 is not high and changed from 1 to 0.
    last_channel_5 = 0;                                                     //Remember current input state.
    aux1_chan = current_time - timer_5;                             //Channel 4 is current_time - timer_4.
  }

   //Channel 6=========================================
  if(PINB & B00100000 ){                                                    //Is input 11 high?
    if(last_channel_6 == 0){                                                //Input 11 changed from 0 to 1.
      last_channel_6 = 1;                                                   //Remember current input state.
      timer_6 = current_time;                                               //Set timer_4 to current_time.
    }
  }
  else if(last_channel_6 == 1){                                             //Input 11 is not high and changed from 1 to 0.
    last_channel_6 = 0;                                                     //Remember current input state.
    aux2_chan = current_time - timer_6;                             //Channel 4 is current_time - timer_4.
  }
}

