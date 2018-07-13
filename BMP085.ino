#include <Wire.h>
#include <BMP085.h>
/*Connection :
 * VCC - +3.3 V (5 V)
 * SDA - A4 (UNO, Nano)
 * SCL - A5 (UNO, Nano)
 * GND - GND
 *Altitude is set to zero, when sensor is powered on. 
 *To set correct value add to altitude variable height 
 *above sea in meters (get it from GPS)
 */

BMP085 dps = BMP085();

long Temperature = 0, Pressure = 0, Altitude = 0;

void setup(void) {
Serial.begin(9600);
Wire.begin();
delay(1000);

dps.init();
}

void loop(void) {
  dps.getPressure(&Pressure);
  dps.getAltitude(&Altitude);
  dps.getTemperature(&Temperature);
  
  Serial.print("  Alt(m):");
  Serial.print(Altitude/100., 2);
  Serial.print("  Pressure(mm Hg):");
  Serial.print(Pressure/133.3);
  Serial.print(" Temp:");
  Serial.println(Temperature*0.1);
  delay(2000);
}
