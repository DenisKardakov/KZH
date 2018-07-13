/**
 * DESCRIPTION
 * 
 * Dust Sensor for SamYoung DSM501
 *   connect the sensor as follows :
 *    Pin 2 of dust sensor PM1      -> Digital 3 (PMW)
 *    Pin 3 of dust sensor          -> +5V 
 *    Pin 4 of dust sensor PM2.5    -> Digital 6 (PWM) 
 *    Pin 5 of dust sensor          -> Ground
 * Datasheet: http://www.samyoungsnc.com/products/3-1%20Specification%20DSM501.pdf
**/
  
  


#define DUST_SENSOR_DIGITAL_PIN_PM10  3
#define DUST_SENSOR_DIGITAL_PIN_PM25  6


//VARIABLES
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



void setup()  
{
   Serial.begin(9600);
  pinMode(DUST_SENSOR_DIGITAL_PIN_PM10,INPUT);
  pinMode(DUST_SENSOR_DIGITAL_PIN_PM25,INPUT);
  //digitalWrite(DUST_SENSOR_DIGITAL_PIN_PM10, HIGH);
  //digitalWrite(DUST_SENSOR_DIGITAL_PIN_PM25, HIGH);
}

void loop()      
{ 
  /*   NEEDS TO BE CALIBRATED CAUSE OUTPUT CONCENRATION IS NEGATIVE
   
  //get PM 2.5 density of particles over 2.5 μm.
  concentrationPM25=(long)getPM(DUST_SENSOR_DIGITAL_PIN_PM25);
  //Serial.print("PM25: ");
 // Serial.println(concentrationPM25);
 // Serial.print("\n");
  //ppmv=mg/m3 * (0.08205*Tmp)/Molecular_mass
  //R = 0.08205   = Universal gas constant in atm·m3/(kmol·K)
  ppmv25=(float)(((concentrationPM25*0.0283168)/100) *  ((0.08205*temp)/0.01))/1000; // ppmv - how many ml (mg) of material is in the 1 m^3
  Serial.print("PPMV25 : ");
  Serial.println(ppmv25);
*/
 
  
 //get PM 1.0 - density of particles over 1 μm.
  concentrationPM10 = getPM(DUST_SENSOR_DIGITAL_PIN_PM10);
 // Serial.print("PM10: ");
 // Serial.println(concentrationPM10);
  //Serial.print("\n");
  //ppmv=mg/m3 * (0.08205*Tmp)/Molecular_mass
  //R = 0.08205   = Universal gas constant in atm·m3/(kmol·K)
  ppmv10=(((concentrationPM10*0.0283168/100) *  (0.08205*temp)/0.01))/1000; // ppmv - how many ml (mg) of material is in the 1 m^3
  Serial.print("PPMV10 : ");
  Serial.println(ppmv10);
 
 
  
}

long getPM(int DUST_SENSOR_DIGITAL_PIN) {

  starttime = millis();

  while (1) {
  
    duration = pulseIn(DUST_SENSOR_DIGITAL_PIN, LOW);
    lowpulseoccupancy += duration;
    endtime = millis();
    
    if ((endtime-starttime) > sampletime_ms)
    {
    ratio = (lowpulseoccupancy-endtime+starttime)/(sampletime_ms*10.0);  // Integer percentage 0=>100
    long concentration = 1.1*pow(ratio,3)-3.8*pow(ratio,2)+520*ratio+0.62; // using spec sheet curve (pcs/283 ml) (1 ft^3 = 28316,85 ml = 0,0283 m^3)
    //Serial.print("lowpulseoccupancy:");
    //Serial.print(lowpulseoccupancy);
    //Serial.print("\n");
   // Serial.print("ratio:");
    //Serial.println(ratio);
    //Serial.print("\n");
    //Serial.print("DSM501A:");
    //Serial.println(concentration);
    //Serial.print("\n");
    
    lowpulseoccupancy = 0;
    return(concentration);    
    }
  }  
}
