/*
Библиотека разработана на основе Tiny_ModBusRTU_Master (Калининым Эдуардом mypractic.ru)
добавил возможность использовать SoftwareSerial
*/

// проверка, что библиотека еще не подключена
#ifndef ArduinoModBusRTU_Slave_h // если библиотека StepMotor не подключена
#define ArduinoModBusRTU_Slave_h // тогда подключаем ее

#include "Arduino.h"

class ArduinoModBusRTU_Slave {

public:
	// конструктор
	ArduinoModBusRTU_Slave(byte adress, byte timeOut,unsigned int * holdingRegTable, unsigned int lengthTable); 	
	ArduinoModBusRTU_Slave(byte adress, byte timeOut, unsigned int * holdingRegTable, unsigned int lengthTable, byte directPin); // конструктор
	void update();  // загрузка данных
	boolean flagRead; // признак данные были считаны
	boolean flagWrite; // признак данные были записаны
	void setAdress(byte adress); // установка адреса	

private:
	byte _adress; // адрес       
	byte _timeOut; // врем¤ тайм-аута в циклах вызова update()
	unsigned int * _holdingRegTable;  // указатель на таблицу регистров
	unsigned int _lengthTable;  // количество регистров
	byte _directPin;  // вывод направлени¤ передачи 
	boolean _noDirectPin;  // признак нет вывода направление 

	unsigned int calculateCRC(byte buffSize); // расчет контрольной суммы 
	void errorFrame();  // ошибка кадра

	byte _frameBuffer[64]; // буфер кадра
	byte _mode; // режим
	byte _timeOutCounter; // счетчик времени тайм-аута
	byte _dataNum; // число данных      
};

#endif