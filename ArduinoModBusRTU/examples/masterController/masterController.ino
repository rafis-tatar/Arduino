/*
    Name:       masterController.ino
    Created:	12.07.2018 9:01:51
    Author:     ARMITS-20310298\hex
*/
#include <SoftwareSerial.h>
#include <ArduinoModBusRTU_Master.h>
#include <TimerOne.h>
#include "localHost.h"

ArduinoModBusRTU_Master _master(8, 30, 13, 2, 3, false);

// 8, 30 - таймауты
// 13 - пин светодиода (встроенного)
// 2 - пин RX
// 3 - пин TX

/*########таблица регистров##
[0] - состояние (1-включен, 2-выключен)
[1] - сигнал (1-включить,2-выключить,)
[2] - состояние команды (0-неопределенно, 1-OK, 2-Выполняется, 3-Ошибка)
###########################*/
unsigned int regTable[3];

int _count = 2; // кол-во хост-контроллеров
int _startAddres = 100; // начальный адрес хост-контроллеров
int interval = 10000; // интервал опроса шины в микросекундах
localHost *hosts[_count]; // массив хост-контроллеров

void setup()
{
	Serial.begin(9600);
	
	pinMode(LED_BUILTIN, OUTPUT);

	Timer1.initialize(interval);
	Timer1.attachInterrupt(UpdateSensorValues,interval);

	for (int i = 0; i < _count; i++)
	{
		hosts[i] = new localHost((byte)(_startAddres + i), 200 + (i * 100));
	}
}

int index = 0;
int mode = 0;

void loop()
{	
	if (_master.state != 1) {
		switch (mode)
		{
		case 0:
			ReadData();		
			break;
		case 1:
			SetData();
			break;
		}
	}
}


void ReadData()
{
	localHost *host = hosts[index];
	_master.read(host->ID, regTable, 0, 3);	
	mode = 1;	
}
void SetData()
{
	localHost *host = hosts[index];

	int val = analogRead(0);
	if (!host->IsOn && host->MinValue <= val)
	{
		_master.writeSingle(host->ID, 1, 1);	
		host->IsOn = true;
		Serial.print("ID: "); Serial.print(host->ID); Serial.println("; On");
	}
	else if (host->IsOn && val < host->MinValue)
	{
		_master.writeSingle(host->ID, 2, 1);
		host->IsOn = false;
		Serial.print("ID: "); Serial.print(host->ID); Serial.println("; Off");
	}

	index++;
	if (index >= _count) index = 0;
	mode = 0;
}
void UpdateSensorValues()
{
	_master.update();
}
