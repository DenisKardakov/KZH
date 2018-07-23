#include <RunningAverage.h>
#define calFactor 1.0/4.8 //value, written on the sensor 
/*
 * Connection to Arduino (Nano, Uno)
 * VCC - 5V
 * GND - GND 
 * OUT - D2
 */


volatile unsigned int count = 0;
byte runCount = 0;
unsigned int countPerRun[30];
unsigned int countPerMinute = 0;
unsigned int averageCountPerMinute = 0;
float radiationValue = 0.0;
int outPin = 2;
unsigned long timePrevious = 0;

RunningAverage RunningAverageCountPerMinute(10); // 10 - is a number of internal array or MAXCNT


void setup() 
  {
    Serial.begin(9600);
    Serial.println("CPM;AV;uSv/h");
    attachInterrupt(0, countPulse, RISING);// if sensor gets an pulse it starts to work
  }

void loop() 
{
  
  if (millis()-timePrevious > 2000)// if since the last measure past more tham 2 seconds
    {
      timePrevious = millis(); // get the time
      byte i;
      for (i=0; i<30; i++) 
        {
          if (i == runCount) 
            {
              countPerRun[i] = count;
            }
          countPerMinute+=countPerRun[i];
        }
      
      RunningAverageCountPerMinute.addValue(count); // adds value to the internal loop to count average
      averageCountPerMinute = RunningAverageCountPerMinute.getFastAverage();// returns average of all counts for the las minute
       
      radiationValue = (float)countPerMinute * (float)calFactor;
      
      
      Serial.print(countPerMinute);
      Serial.print(";");
      Serial.print(averageCountPerMinute);
      Serial.print(";");
      Serial.println(radiationValue);
       
      runCount++;// counts 1 minute by 2 seconds
      if(runCount > 29)// so 2*30 = 60 seconds = 1 minute 
        {
          runCount = 0;
        }
       
      count = 0;// zero the counter every 2 seconds
      countPerMinute = 0;
    }
   
}

void countPulse()
  {
    count++;
    detachInterrupt(0);
    while(digitalRead(outPin)==1){}
    attachInterrupt(0,countPulse,RISING);
  }
