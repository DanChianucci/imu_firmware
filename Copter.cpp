#include "Copter.h"
#include "wire.h"

//The setup function is called once at startup of the sketch
void setup()
{

	Serial.begin(19200);

//	Serial.flush();
//
//	while(Serial.available()>0 || Serial.peek()!='?')
//	{
//		Serial.read();
//	}
//	Serial.println("I am an AHRSuIMU");
//	Serial.flush();
	Wire.begin();
	imu.init();

}

int PERIOD=20;
long waitTime=0;

void loop()
{
	long startloop = millis();

	imu.Update();
	imu.getData(qV,aV,gV,mV);

	if(Serial.available() && Serial.peek()=='g')
		printQuat();

	else if(Serial.available() && Serial.peek()=='a')
		printData();

	while(Serial.available())
		Serial.read(); //empty the buffer


	waitTime = PERIOD-(millis()-startloop);
	if(waitTime>0)
	{
		delay(waitTime);
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

void printQuat()
{
	for(int i=0; i < 4; i++)
		{
			Serial.print(qV[i]);
			Serial.print(",");
		}
	Serial.println(".");
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

	Serial.println(waitTime);
}

