#include <TimerOne.h>
#include <ArduinoModBusRTU_Slave.h>

/*
    Name:       localController.ino
    Created:	12.07.2018 8:47:38
    Author:     hex
*/


/*########таблица регистров##
[0] - состояние (1-включен, 2-выключен)
[1] - сигнал (1-включить,2-выключить,)
[2] - состояние команды (0-неопределенно, 1-OK, 2-Выполняется, 3-Ошибка,)
###########################*/
unsigned int regTable[3]{ 0,0,0 };

//100;101; - адреса модулей
ArduinoModBusRTU_Slave slave(101, 8, regTable, 3); 

int interval = 10000;
void setup()
{
	Serial.begin(9600);
	Timer1.initialize(interval);
	Timer1.attachInterrupt(UpdateSlave, interval);
	pinMode(LED_BUILTIN, OUTPUT);	
}
bool ledOn = false;
void loop()
{		
	if (!ledOn && regTable[1] == 1)
	{		
		noInterrupts();
		regTable[2] = 2;
		ActionOn(); //todo somthin
		ledOn = true;
		regTable[2] = 1;
		regTable[0] = 1;
		interrupts();
	}
	else if ( ledOn && regTable[1] == 2)
	{		
		noInterrupts();
		regTable[2] = 2;
		ActionOff();//todo somthin
		ledOn = false;
		regTable[2] = 1;
		regTable[0] = 2;
		interrupts();
	}
}

void ActionOn()
{
	digitalWrite(LED_BUILTIN, HIGH);	
}
void ActionOff()
{
	
	digitalWrite(LED_BUILTIN, LOW);	
}

void UpdateSlave() {	
	slave.update();		
}

