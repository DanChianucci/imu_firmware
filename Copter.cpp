#include "Copter.h"

#include <avr/interrupt.h>

#include "wire.h"

//The setup function is called once at startup of the sketch
void setup()
{
	Serial.begin(115200);
	Wire.begin();
	imu.init();
}

#define PERIOD 20

void loop() {
	int startloop = millis();

	imu.Update();
	imu.getData(qV,aV,gV,mV);

	for(int i=0; i < 4; i++)
	{
		Serial.print(qV[i]);
		Serial.print(",");
	}

	for(int i=0; i < 3; i++)
	{
		Serial.print(aV[i]);
		Serial.print(",");
	}
	for(int i=0; i < 3; i++)
	{
		Serial.print(mV[i]);
		Serial.print(",");
	}
	for(int i=0; i < 3; i++)
	{
		Serial.print(gV[i]);
		Serial.print(",");
	}
	Serial.println("");

	int wait = PERIOD-(millis()-startloop);
	if(wait>0)
	{
		delay(wait);
	}
}

