/*************************************************************************************************
  LaCrosse_TX23.h - Library for reading LaCrosse TX23 anemometer data.
  
  LaCrosse TX23 is a wind speed and direction sensor. It uses 3 wires for communication and power:
  Pin1	Brown(Black)	DATA
  Pin2	Red				Vcc
  Pin3	Green			N/C
  Pin4	Yellow			GND
  
  DATA pin is to be connected directly to one of Arduino ports.
  
  
  
  Created by Krzysztof Egzmont, 06.11.2016.
  Based on https://www.john.geek.nz/2012/08/la-crosse-tx23u-anemometer-communication-protocol/
  Big thanks for John.
  
  Donations: 
  Bitcoin - 1JrHQwJWt1JNZepT1EXqPSioXcwD227nnU
  Ethereum - 0xFC9713013e78f6F14bA5064E36f875dbbB32B94a
  
  BSD license, all text above must be included in any redistribution
*************************************************************************************************/

#include <LaCrosse_TX23_2.h>

LaCrosse_TX23_2::LaCrosse_TX23_2(int pin)
{
pinMode(pin, INPUT);
_pin = pin;
}

void LaCrosse_TX23_2::pullBits(void *dst, bool *src, int count)
{
	uint8_t * d = (uint8_t*) dst;
	for (int i=0; i<count; i++)
	{
		d[i/8] |= src[i]<<(i%8);
	}	
}

bool LaCrosse_TX23_2::read(float &speed,float &gust, int &direction)
{
	speed = 0;
	gust=0;
	direction = 0;

	digitalWrite(_pin,LOW);
	pinMode(_pin,OUTPUT);
	delay(500);
	pinMode(_pin,INPUT);
	pulseIn(_pin,LOW);

	unsigned bitLen = 1200;
	bool data[50];
	bool lastState = 1;
	unsigned long start = micros();
	

	for(unsigned long t = 0; t<50000; t = micros()-start)
	{
		bool state = digitalRead(_pin);
		unsigned bitNum = t/bitLen;
		if(t%bitLen>bitLen/2) data[bitNum] = state;
		if(state!=lastState)
		{
			unsigned delta = t%bitLen;
			if(delta<100)
				start -= delta;
			else if(delta>900)
				start += delta;
			lastState = state;
		}
	}
	
	unsigned bitLen2 = 1200;
	bool data2[50];
	bool lastState2 = 1;
	delay(20);

	unsigned long start2 = micros();
	
	for(unsigned long t2 = 0; t2<50000; t2 = micros()-start2)
	{
		bool state2 = digitalRead(_pin);
		unsigned bitNum2 = t2/bitLen2;
		if(t2%bitLen2>bitLen2/2) data2[bitNum2] = state2;
		if(state2!=lastState2)
		{
			unsigned delta2 = t2%bitLen2;
			if(delta2<100)
				start2 -= delta2;
			else if(delta2>900)
				start2 += delta2;
			lastState2 = state2;
		}
	}

	uint8_t ctr = 0; pullBits(&ctr,data+0,5);
	uint8_t dir = 0; pullBits(&dir,data+5,4);
	uint16_t spd = 0; pullBits(&spd,data+9,12);

	uint16_t spd_gust = 0; pullBits(&spd_gust,data2+8,11);

	uint8_t sum = 0; pullBits(&sum,data+21,4);
	uint8_t ndir = 0; pullBits(&ndir,data+25,4);
	ndir ^= 0x0f;
	uint16_t nspd = 0; pullBits(&nspd,data+29,12);
	nspd ^= 0x0fff;

	uint8_t csum = 0x0f & (dir + (spd&0x0f) + ((spd>>4)&0x0f) + ((spd>>8)&0x0f));

	if(ctr!=27) return false;
	if(csum!=sum) return false;
	if(spd!=nspd || dir!=ndir) return false;

	speed = spd/10.0;
	gust = spd_gust/10.0;
	direction = dir;

	return true;
}
