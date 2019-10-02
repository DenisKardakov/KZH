#define BLYNK_PRINT Serial
#define REDPIN 6
#define GREENPIN 3
#define BLUEPIN 5
#define FADESPEED 5     // чем выше число, тем медленнее будет fade-эффект
#include <MHZ19.h>
#include <ESP8266_Lib.h>
#include <BlynkSimpleShieldEsp8266.h>
#include <SoftwareSerial.h>
#include <TimeLib.h>
#include <DS1307RTC.h>
#include <SD.h>
#include <SPI.h>  
#include <string.h>
#include <WiFi.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Wire.h> //BH1750 IIC Mode 
#include <math.h> 

// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
const char auth[] = "170a9b98d6d643969f3ee0ef214a8f8b";

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "ZyXEL_KEENETIC_LITE_72EF60";//Student Design Bureau 
char pass[] = "0660627225";//AbCD8168 0660627225

//Initializing CO2 sensor (MHZ-19)
#define mhzSerial Serial2
MHZ19 myMHZ19;
unsigned long getDataTimer = 0; 

// I2C bus adress for light intensity sensor (BH1750)
int BH1750address = 0x23; //Устанавливаем i2c address
byte buff[2];

//Initializing temp/hum/alt/pressure sensor (ds18s20)
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme;

//Initializing wi-fi module (ESP-01)
#define EspSerial Serial3
// Your ESP8266 baud rate:
#define ESP8266_BAUD 115200
ESP8266 wifi(&EspSerial);

//variables for dust sensor
#define DUST_SENSOR_DIGITAL_PIN_PM10  3
#define DUST_SENSOR_DIGITAL_PIN_PM25  6
unsigned long duration;
unsigned long starttime;
unsigned long endtime;
unsigned long sampletime_ms = 5000;  // 30 second measurement length (use 5~30 second measurement time)
unsigned long lowpulseoccupancy = 0; // data directly from sensor
float ratio = 0;
long concentrationPM25 = 0; //(pcs/283 ml)
long concentrationPM10 = 0; //(pcs/283 ml)
int temp=20; //external temperature, if you can replace this with a DHT11 or better 
float ppmv10, ppmv25;

//variables for watch
const char *monthName[12] = {
  "Jan", "Feb", "Mar", "Apr", "May", "Jun",
  "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
};
 char timestamp[30];
 char data[30];
 //tmElements_t tm; // time object
// File myFile;

double eval = 0;
//BlynkTimer timer;
BlynkTimer lux_timer;
BlynkTimer co2_timer;
BlynkTimer temp_timer;
//BlynkTimer dust_timer;
BlynkTimer hum_timer;
BlynkTimer pres_timer;
BlynkTimer eval_calc;
/*void form_stamp(int value, char* name)
{
  sprintf(data, "%s%d %s ", data, value, name);
} */
void getTemp()
{
  double t = bme.readTemperature(); 
  Blynk.virtualWrite(V0, t);
  Serial.print("Temperature = ");
	Serial.print(t);
	Serial.println("*C");
  eval += t;
   //sprintf(data + strlen(data), "t = %2d C ", t);
   //Serial.println(data);
}

void getHum()
{
  double h = bme.readHumidity(); 
  Blynk.virtualWrite(V3, h);
  Serial.print("Humidity = ");
	Serial.print(h);
	Serial.println("%");
  eval += h;
}

void getPres()
{
  double p = bme.readPressure(); 
  Blynk.virtualWrite(V5, p/133.3);// сonvert hectopascals to mmHg
  Serial.print("Pressure = ");
	Serial.print(p/133.3);
	Serial.println("mmHg");
}


void getCO2()
{
  if (millis() - getDataTimer >= 2000)                    // Check if interval has elapsed (non-blocking delay() equivilant)
    {
        int CO2;
        //char name_co2 [3];
        //sprintf(name_co2, "ppm");                                       // Buffer for CO2
        CO2 = myMHZ19.getCO2();                             // Request CO2 (as ppm) 
        Blynk.virtualWrite(V1, CO2);
        //form_stamp(CO2, name_co2);                               
        getDataTimer = millis();
        eval += CO2;
    }
   
   
}

void start_bh1750() // lux sensor
{
  Wire.beginTransmission(BH1750address);
  Wire.write(0x10);//1lx reolution 120ms
  Wire.endTransmission();
}
void getLux()
{
  int i=0;
  uint16_t lux=0;
  Wire.beginTransmission(BH1750address);
  Wire.requestFrom(BH1750address, 2);
  while(Wire.available()) //
  {
    buff[i] = Wire.read();  // receive one byte
    i++;
  }
  if(2==i)
  {
    lux=((buff[0]<<8)|buff[1])/1.2;
    Serial.print(lux,DEC);     
    Serial.println("[lx]"); 
    Blynk.virtualWrite(V2, lux);
    eval += lux;
  }
  Wire.endTransmission();  
}

void start_bme280() // Temperature, Humidity, Pressure and Altitude sensor
{
  if (!bme.begin(0x76)) {
		Serial.println("Could not find a valid BME280 sensor, check wiring!");
		while (1);
	}
}


/*long getPM(int DUST_SENSOR_DIGITAL_PIN)// this function returns the concentration of particles
{
  starttime = millis();

    while (1) {
    
      duration = pulseIn(DUST_SENSOR_DIGITAL_PIN, LOW);
      lowpulseoccupancy += duration;
      endtime = millis();
      
      if ((endtime-starttime) > sampletime_ms)
      {
      ratio = (lowpulseoccupancy-endtime+starttime)/(sampletime_ms*10.0);  // Integer percentage 0=>100
      long concentration = 1.1*pow(ratio,3)-3.8*pow(ratio,2)+520*ratio+0.62; // using spec sheet curve (pcs/283 ml) (1 ft^3 = 28316,85 ml = 0,0283 m^3)
      lowpulseoccupancy = 0;
      return(concentration);    
      }
    }  
}

void getDust()
{
  //get PM 1.0 - density of particles over 1 μm.
  concentrationPM10 = getPM(DUST_SENSOR_DIGITAL_PIN_PM10);
  concentrationPM25=(long)getPM(DUST_SENSOR_DIGITAL_PIN_PM25);
  ppmv25=(float)(((concentrationPM25*0.0283168)/100) *  ((0.08205*temp)/0.01))/1000; // ppmv - how many ml (mg) of material is in the 1 m^3
  ppmv10=(((concentrationPM10*0.0283168/100) *  (0.08205*temp)/0.01))/1000; // ppmv - how many ml (mg) of material is in the 1 m^3
  if(ppmv25<0)
  {

  } else 
  {
    Blynk.virtualWrite(V4, ppmv25);
  }
  
  Serial.print("PPMV10 : ");
  Serial.println(ppmv10);
  Serial.print("PPMV25 : ");
  Serial.println(ppmv25);
}*/

/*
bool setTime(const char *str)
{
  int Hour, Min, Sec;

  if (sscanf(str, "%d:%d:%d", &Hour, &Min, &Sec) != 3) return false;
  tm.Hour = Hour;
  tm.Minute = Min;
  tm.Second = Sec;
  return true;
}

bool setDate(const char *str)
{
  char Month[12];
  int Day, Year;
  uint8_t monthIndex;

  if (sscanf(str, "%s %d %d", Month, &Day, &Year) != 3) return false;
  for (monthIndex = 0; monthIndex < 12; monthIndex++) {
    if (strcmp(Month, monthName[monthIndex]) == 0) break;
  }
  if (monthIndex >= 12) return false;
  tm.Day = Day;
  tm.Month = monthIndex + 1;
  tm.Year = CalendarYrToTm(Year);
  return true;
}

void set_time_date()
{
  bool parse=false;
  bool config=false;

  // get the date and time the compiler was run
  if (setDate(__DATE__) && setTime(__TIME__)) {
    parse = true;
    // and configure the RTC with this info
    if (RTC.write(tm)) {
      config = true;
    }
  }

   // while (!Serial) ; // wait for Arduino Serial Monitor
  delay(200);
  if (parse && config) {
    Serial.print("DS1307 configured Time=");
    Serial.print(__TIME__);
    Serial.print(", Date=");
    Serial.println(__DATE__);
  } else if (parse) {
    Serial.println("DS1307 Communication Error :-{");
    Serial.println("Please check your circuitry");
  } else {
    Serial.print("Could not parse info from the compiler, Time=\"");
    Serial.print(__TIME__);
    Serial.print("\", Date=\"");
    Serial.print(__DATE__);
    Serial.println("\"");
  }
}

void print2digits(int number) // formats the serial output for dates
{
  if (number >= 0 && number < 10) {
    Serial.write('0');
  }
  Serial.print(number);
}

void print_date_time()
{
   if (RTC.read(tm)) 
   {
    Serial.print("Ok, Time = ");
    print2digits(tm.Hour);
    Serial.write(':');
    print2digits(tm.Minute);
    Serial.write(':');
    print2digits(tm.Second);
    Serial.print(", Date (D/M/Y) = ");
    Serial.print(tm.Day);
    Serial.write('/');
    Serial.print(tm.Month);
    Serial.write('/');
    Serial.print(tmYearToCalendar(tm.Year));
    Serial.println();
  
    sprintf(timestamp, "%02d:%02d:%02d %2d/%2d/%2d \n", tm.Hour,tm.Minute,tm.Second,tm.Month,tm.Day,tmYearToCalendar(tm.Year));
    
   }
    else 
    {
      if (RTC.chipPresent()) 
      {
        set_time_date();
      } else 
      {
        Serial.println("DS1307 read error!  Please check the circuitry.");
        Serial.println();
      }
     // delay(9000);
    }
 //delay(1000);
}

void dateTime(uint16_t* date, uint16_t* time) // this function makes correct timestamp, when file is created
{
  
 // return date using FAT_DATE macro to format fields
 *date = FAT_DATE(tmYearToCalendar(tm.Year), tm.Month, tm.Day);

 // return time using FAT_TIME macro to format fields
 *time = FAT_TIME(tm.Hour, tm.Minute, tm.Second);
}

void start_SD()
{
  Serial.print("Initializing SD card...");

  if (!SD.begin(53)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");
  SdFile::dateTimeCallback(dateTime);
}*/



/*void data_stamp()
{
  myFile = SD.open("data.txt", FILE_WRITE);
  if (myFile)
  {
    Serial.println("Writing to the file");
    myFile.println(timestamp);
    myFile.println(data);
    myFile.close();
    Serial.println(data); 
    memset(data, 0, sizeof data);
  }else {
    // if the file didn't open, print an error:
    Serial.println("error opening file");
  }
}*/

void getEval()
{
   Serial.println(eval); 
   if(eval>600 && eval<3000)
   {
      analogWrite(GREENPIN, 255);
      analogWrite(BLUEPIN, 0);
      analogWrite(REDPIN, 0);
   }
    if((eval<600 && eval>248) || (eval>3000 && eval<5086))
   {
      analogWrite(GREENPIN, 255);
      analogWrite(BLUEPIN, 0);
      analogWrite(REDPIN, 255);
   }
   if(eval<248 || eval>5086)
   {
      analogWrite(GREENPIN, 0);
      analogWrite(BLUEPIN, 0);
      analogWrite(REDPIN, 255);
   }
   eval = 0;
}

void setup()
{
  // Debug console
  Wire.begin();
  Serial.begin(115200);
  
  //connectionHandlerTimer.setInterval(100L, ConnectionHandler);
  //connectionState = CONNECT_TO_BLYNK;
    // Set ESP8266 baud rate
  EspSerial.begin(ESP8266_BAUD);
  // co2 sensor port
  mhzSerial.begin(9600);
  delay(10);
  // start blynk
  Blynk.begin(auth, wifi, ssid, pass);
  //start co2 sensor
  myMHZ19.begin(mhzSerial);                               // *Important, Pass your Stream reference 
  myMHZ19.autoCalibration(); 
  //start temperature sensor 
  //set_time_date();
  //start_SD();
  // Light sensor startup routine
  start_bh1750();
  //delay(1000);
  start_bme280();
  //pinMode(DUST_SENSOR_DIGITAL_PIN_PM10,INPUT);
  //pinMode(DUST_SENSOR_DIGITAL_PIN_PM25,INPUT);
  // LED strip light pin declaration
  pinMode(REDPIN, OUTPUT);
  pinMode(GREENPIN, OUTPUT);
  pinMode(BLUEPIN, OUTPUT);

//timer.setInterval(1100L, CheckConnection);
lux_timer.setInterval(2000L, getLux);
co2_timer.setInterval(2000L, getCO2);
temp_timer.setInterval(2000L, getTemp);
//dust_timer.setInterval(2000L, getDust);
hum_timer.setInterval(2000L, getHum);
pres_timer.setInterval(2000L, getPres);
eval_calc.setInterval(4000L, getEval); 
}

void loop()
{
  Blynk.run();
  //connectionHandlerTimer.run();
  lux_timer.run();
  co2_timer.run();
  temp_timer.run();
  //dust_timer.run();
  hum_timer.run();
  eval_calc.run();
  pres_timer.run();
  
  //analogWrite(REDPIN, 255);
 
}
