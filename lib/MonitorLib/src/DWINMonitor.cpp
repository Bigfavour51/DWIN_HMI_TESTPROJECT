#include "DWINMonitor.h"




void Data_Arduino_to_Display()
{
  int t = 10;
  int h = 20;
  int p = 30;
  int v = 40;
  int ap =20;
  int e = 50;
  /*------Print data to Serial Monitor------*/
   Serial.print("Temperature = ");
  Serial.print(t);
  Serial.println(" °C");

  Serial.print("Humidity = ");
  Serial.print(h);
  Serial.println(" %");

  Serial.print("Pressure = ");
  Serial.print(p);
  Serial.println(" hPa");

  Serial.print("Approx. Altitude = ");
  Serial.print(v);
  Serial.println(" m");

  Serial.print("Atmospheric Pressure = ");
  Serial.print(ap);
  Serial.println(" °C");
  Serial.println();

  Serial.print("Energy = ");
  Serial.print(e);
  Serial.println(" KWh");
  Serial.println();
  

  Temperature[6] = highByte(t);
  Temperature[7] = lowByte(t);
  Serial.write(Temperature, 8);
  Humidity[6] = highByte(h);
  Humidity[7] = lowByte(h);
  Serial.write(Humidity, 8);
  Power[6] = highByte(p);
  Power[7] = lowByte(p);
  Serial.write(Power, 8);
  Voltage[6] = highByte(v);
  Voltage[7] = lowByte(v);
  Serial.write(Voltage, 8);
  ATpressure[6] = highByte(v);
  ATpressure[7] = lowByte(v);
  Serial.write(ATpressure, 8);
  Energy[6] = highByte(ap);
  Energy[7] = lowByte(ap);
  Serial.write(Energy, 8);
}


