#include <Wire.h>
#include <TinyGPS++.h>
#include <genieArduino.h>
#include <Adafruit_MPL115A2.h>
#include <Sensirion.h>
#include "RTClib.h"

// Pin assignement
#define RESETLINE       4
// LED
#define VIOLET_LED_PIN  9
#define BLUE_LED_PIN    13
// SHT11: temperature, humidity, dewpoint
#define SHT11_DATA_PIN  2
#define SHT11_CLOCK_PIN 3

// Magnetometer
#define MAGNETOMETER_ZOOM_OUT_BUTTON 8
#define MAGNETOMETER_ZOOM_IN_BUTTON 7
#define MAGNETOMETER_ADDR 0x1E
#define SCOPE_MAX 50
#define MAGNETOMETER_MIN -2048
#define MAGNETOMETER_MAX 2048

static int8_t magnetometer_gain = 1;
static int magnetometer_z = 0;

// Temperature and Preassure
Adafruit_MPL115A2 mpl115a2;
RTC_DS1307 rtc;

static const uint32_t GPSBaud = 38400;

TinyGPSPlus gps;
Genie genie;
Sensirion sht = Sensirion(SHT11_DATA_PIN, SHT11_CLOCK_PIN);

char day_string[3];
char year_string[5];

char gps_num_satalites[4];
char gps_lat_string[20];
char gps_lon_string[20];
char gps_altitude[20];

#define VIDEO_MAX 30
#define VIDEO_TIMEOUT 80
uint8_t video_index = 0;

static int prev_dow = -1;
static int prev_month = -1;
static int prev_day = -1;
static int prev_year = -1;

static int prev_gps_valid = -1;

void setup() {
  DateTime now;
  Serial.begin(115200);
  Serial1.begin(115200);
  
  genie.Begin(Serial1);   // Use Serial0 for talking to the Genie Library, and to the 4D Systems display
  genie.AttachEventHandler(myGenieEventHandler); // Attach the user function Event Handler for processing events
  
  pinMode(RESETLINE, OUTPUT);
  
  digitalWrite(RESETLINE, 1);  // Reset the Display via D4
  delay(100);
  digitalWrite(RESETLINE, 0);  // unReset the Display via D4
  delay (5000); //let the display start up after the reset (This is important)
  genie.WriteContrast(15); // 1 = Display ON, 0 = Display OFF.
  //For uLCD43, uLCD-70DT, and uLCD-35DT, use 0-15 for Brightness Control, where 0 = Display OFF, though to 15 = Max Brightness ON.
  
  //Serial.println(F("Happy Birthday Ying!"));
  //Serial.println(F("Credits:"));
  //Serial.println(F("\tGPS Code by Mikal Hart"));  
  
  // Setup GPS
  // The serial connection to the GPS device
  Serial3.begin(GPSBaud);
  
  //Setup Magnetometer
  Wire.begin();
  //Put the HMC5883 IC into the correct operating mode
  Wire.beginTransmission(MAGNETOMETER_ADDR); //open communication with HMC5883
  Wire.write(0x02); //select mode register
  Wire.write(0x00); //continuous measurement mode
  Wire.endTransmission();
  
  Wire.beginTransmission(MAGNETOMETER_ADDR);
  Wire.write(0x01); //select register 3, X MSB register
  Wire.write(0x00);
  Wire.endTransmission();  
  
  //LED
  pinMode(VIOLET_LED_PIN, OUTPUT);
  analogWrite(VIOLET_LED_PIN, 255);
  pinMode(BLUE_LED_PIN, OUTPUT);
  analogWrite(BLUE_LED_PIN, 255);
  
  //Temperature Preassure
  mpl115a2.begin();
  
  //RTC
  rtc.begin();
  now = rtc.now();
  snprintf(day_string, sizeof(day_string), "%d", now.day());
  snprintf(year_string, sizeof(year_string), "%d", now.year());  
}

void loop(){ 
  static long waitPeriod = millis();
  static long video_wait = millis();
  update_gps();
  update_magnetometer();
  update_light_sensor();
  
  if (millis() >= video_wait){
    video_wait = millis() + VIDEO_TIMEOUT;
    if (video_index >= VIDEO_MAX){
      video_index = 0;
    }
    else {
      video_index++;
    }
    //Start the video
    genie.WriteObject(GENIE_OBJ_VIDEO, 0, video_index);
  }
    
  if (millis() >= waitPeriod){
    waitPeriod = millis() + 1000; // rerun this code to update Cool Gauge and Slider in another 50ms time.
    update_pt();
    update_datetime();
  }
  genie.DoEvents(); // This calls the library each loop to process the queued responses from the display
}

void update_light_sensor() {
  int light_sensor_value = analogRead(A0);
  //Serial.print("light sensor: ");
  //Serial.println(light_sensor_value);
  light_sensor_value = map(light_sensor_value, 1023, 0, 0, 100);
  genie.WriteObject(GENIE_OBJ_GAUGE, 0, light_sensor_value); 
}

void update_gps(){
  while (Serial3.available() > 0) {
    if (gps.encode(Serial3.read())) {
      if (gps.location.isValid()){
        snprintf(gps_num_satalites, sizeof(gps_num_satalites), "%d", gps.satellites.value());
        dtostrf(gps.location.lat(), 10, 6, gps_lat_string);
        //snprintf(gps_lat_string, sizeof(gps_lat_string), "%f", gps.location.lat());
        dtostrf(gps.location.lng(), 10, 6, gps_lon_string);
        //snprintf(gps_lon_string, sizeof(gps_lon_string), "f", gps.location.lng());
        dtostrf(gps.altitude.meters(), 10, 6, gps_altitude);
        //snprintf(gps_altitude, sizeof(gps_altitude), "%f", gps.altitude.meters());
        
        genie.WriteStr(7, "True");

        genie.WriteStr(8, gps_num_satalites);
        genie.WriteStr(9, gps_lat_string);
        genie.WriteStr(11, gps_lon_string);
        genie.WriteStr(13, gps_altitude);
        prev_gps_valid = gps.location.isValid();        
      } else {
        genie.WriteStr(7, "False");
        prev_gps_valid = gps.location.isValid();        
      }
    }
  }
}

void update_magnetometer(){
  int x,y,z; //triple axis data  
  uint16_t zval;

  //Tell the HMC5883 where to begin reading data
  Wire.beginTransmission(MAGNETOMETER_ADDR);
  Wire.write(0x03); //select register 3, X MSB register
  Wire.endTransmission();
  
  //Read data from each axis, 2 registers per axis
  Wire.requestFrom(MAGNETOMETER_ADDR, 6);
  if (6 <= Wire.available()){
    x = Wire.read()<<8; //X msb
    x |= Wire.read(); //X lsb
    z = Wire.read()<<8; //Z msb
    z |= Wire.read(); //Z lsb
    y = Wire.read()<<8; //Y msb
    y |= Wire.read(); //Y lsb
  }
  Serial.print("Z = ");
  Serial.print(z);
  if (z >= MAGNETOMETER_MIN && z <= MAGNETOMETER_MAX) {
    magnetometer_z = z;
  }

  zval = map(magnetometer_z, MAGNETOMETER_MIN, MAGNETOMETER_MAX, 0, SCOPE_MAX);
  Serial.print(" mapped z = ");
  Serial.println(zval);
  genie.WriteObject(GENIE_OBJ_SCOPE, 0, zval);
  analogWrite(BLUE_LED_PIN, map(zval, 0, SCOPE_MAX, 255, 0));
}

void update_pt(){
  float pressureKPA = 0, temperatureC = 0;  
  uint16_t p;
  uint16_t t;
  mpl115a2.getPT(&pressureKPA, &temperatureC);
  p = (uint16_t) pressureKPA;
  t = ((uint16_t) (temperatureC + .5));
  //Serial.print("temp: ");
  //Serial.println(t);
  //Serial.print("preassure: ");
  //Serial.println(p);
  t = map (t, -10, 40, 0, 50);
  p = map (p, 50, 110, 0, 60);
  
  genie.WriteObject(GENIE_OBJ_THERMOMETER, 0, t);
  genie.WriteObject(GENIE_OBJ_ANGULAR_METER, 0, p);
  
  unsigned int raw_data;
  float humidity;
  uint16_t humidity_int;
  sht.measHumi(&raw_data);
  humidity = sht.calcHumi(raw_data, temperatureC);
  humidity_int = ((uint16_t) (humidity + .5));
  genie.WriteObject(GENIE_OBJ_ANGULAR_METER, 1, humidity_int);
  //Serial.print("humidity: ");
  //Serial.println(humidity_int);
}

void update_datetime(){
  DateTime now = rtc.now();
  genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0, now.hour());
  genie.WriteObject(GENIE_OBJ_LED_DIGITS, 1, now.minute());
    switch (now.dayOfWeek()){
      case (0):  //Sunday
        genie.WriteStr(1, "Sunday");    
        break;
      case (1):
        genie.WriteStr(1, "Monday");    
        break;
      case (2):
        genie.WriteStr(1, "Tuesday");    
        break;
      case (3):
        genie.WriteStr(1, "Wednesday");    
        break;
      case (4):
        genie.WriteStr(1, "Thursday");    
        break;
      case (5):
        genie.WriteStr(1, "Friday");    
        break;
      case (6):
        genie.WriteStr(1, "Saturday");    
        break;
      default:
        genie.WriteStr(1, "WTF");
        break;
    }
    prev_dow = now.dayOfWeek();

    switch (now.month()){
      case (1):
        genie.WriteStr(2, "January");
        break;
      case (2):
        genie.WriteStr(2, "February");
        break;
      case (3):
        genie.WriteStr(2, "March");    
        break;
      case (4):
        genie.WriteStr(2, "April");    
        break;
      case (5):
        genie.WriteStr(2, "May");    
        break;
      case (6):
        genie.WriteStr(2, "June");    
        break;
      case (7):
        genie.WriteStr(2, "July");    
        break;
      case (8):
        genie.WriteStr(2, "August");    
        break;
      case (9):
        genie.WriteStr(2, "September");    
        break;
      case (10):
        genie.WriteStr(2, "October");    
        break;
      case (11):
        genie.WriteStr(2, "November");    
        break;
      case (12):
        genie.WriteStr(2, "December");    
        break;
      default:
        genie.WriteStr(2, "End of days");
    }
    prev_month = now.month();

    snprintf(day_string, sizeof(day_string), "%d", now.day());
    genie.WriteStr(3, day_string);
    prev_day = now.day();

    snprintf(year_string, sizeof(year_string), "%d", now.year());
    genie.WriteStr(4, year_string);  
    prev_year = now.year();    
}

// LONG HAND VERSION, MAYBE MORE VISIBLE AND MORE LIKE VERSION 1 OF THE LIBRARY
void myGenieEventHandler(void)
{
  genieFrame Event;
  genie.DequeueEvent(&Event);
  //Serial.println("Dequeue event!");

  int slider_val = 0;

  //If the cmd received is from a Reported Event (Events triggered from the Events tab of Workshop4 objects)
  if (Event.reportObject.cmd == GENIE_REPORT_EVENT){
    switch (Event.reportObject.object){
      case (GENIE_OBJ_SLIDER):
      if (Event.reportObject.index == 0){
        analogWrite(VIOLET_LED_PIN, map(genie.GetEventData(&Event), 0, 100, 255, 0));
      }
      break;
      case (GENIE_OBJ_FORM):
        prev_dow = -1;
        prev_month = -1;
        prev_day = -1;
        prev_year = -1;
          
        prev_gps_valid = -1;      
      break;
      case (GENIE_OBJ_USERBUTTON):
      //Serial.print("Button: ");
      //Serial.println(Event.reportObject.index);
      if (Event.reportObject.index == MAGNETOMETER_ZOOM_OUT_BUTTON){
        if (magnetometer_gain > 0){
          magnetometer_gain--;
          Wire.beginTransmission(MAGNETOMETER_ADDR);
          Wire.write(0x02);
          Wire.write(magnetometer_gain << 5);
          Wire.endTransmission();          
        }
      } else if (Event.reportObject.index == MAGNETOMETER_ZOOM_IN_BUTTON){
        if (magnetometer_gain < 7){
          magnetometer_gain++;
          Wire.beginTransmission(MAGNETOMETER_ADDR);
          Wire.write(0x02);
          Wire.write(magnetometer_gain << 5);
          Wire.endTransmission();          
        }          
      }
      break;
    }
  }
}


