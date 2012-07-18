#include "Copter.h"

#include <avr/interrupt.h>

#include "wire.h"

//The setup function is called once at startup of the sketch
void setup()
{
	Serial.begin(19200);
	Wire.begin();
	imu.init();
}

int PERIOD=40;


void loop() {
	long startloop = millis();

	imu.Update();
	imu.getData(qV,aV,gV,mV);
	printData();
	getCommands();

	long wait = PERIOD-(millis()-startloop);

	if(wait>0)
	{
		delay(wait);
	}
}

void getCommands()
{
	char buffer[10]={};
	float amount=0;
	Serial.flush();
	if(Serial.available())
	{
		Serial.readBytesUntil(':',buffer,10);
		amount=Serial.parseFloat();
	}
	else
		return;
	String cmd = String(buffer);
	cmd=cmd.substring(0,cmd.indexOf(':'));

	if(cmd.equals("chF"))
	{
		Serial.println("Changing Frequency: ");

		PERIOD=(int)(1000/amount);
		imu.sampleFreq=amount;
		imu.halfT=.5/amount;

		Serial.print("\t Period:     "); Serial.println(PERIOD);
		Serial.print("\t SampleFreq: "); Serial.println(imu.sampleFreq);
		Serial.print("\t HalfT: "); Serial.println(imu.halfT);

	}

	else if(cmd.equals("chB"))
	{
		Serial.print("Changing Beta: "); Serial.println(amount);
		imu.beta=amount;
	}
	else if(cmd.equals("chKp"))
	{
		Serial.print("Changing Kp: "); Serial.println(amount);
		imu.Kp=amount;
	}
	else if(cmd.equals("chKi"))
	{
		Serial.print("Changing Ki: "); Serial.println(amount);
		imu.Ki=amount;
	}
	else
	{
		Serial.println(cmd +" Not Recognized");
		return;
	}

	Serial.println("Restart Commencing");
	Serial.readStringUntil('\n');
	Serial.flush();
	delay(3000);
	imu.reset();
}


void printData()
{
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
}

