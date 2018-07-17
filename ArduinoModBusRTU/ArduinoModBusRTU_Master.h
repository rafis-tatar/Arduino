/*
Библиотека разработана на основе Tiny_ModBusRTU_Master (Калининым Эдуардом mypractic.ru)
добавил возможность использовать SoftwareSerial
*/
#include<SoftwareSerial.h>

// проверка, что библиотека еще не подключена
#ifndef ArduinoModBusRTU_Master_h // если библиотека StepMotor не подключена
#define ArduinoModBusRTU_Master_h // тогда подключаем ее

class ArduinoModBusRTU_Master {

public:
	// конструктор
	ArduinoModBusRTU_Master(byte timeOutTransmit, byte timeOutRecieve);
	// конструктор
	ArduinoModBusRTU_Master(byte timeOutTransmit, byte timeOutRecieve, byte directPin);

	// конструктор
	ArduinoModBusRTU_Master(byte timeOutTransmit, byte timeOutRecieve, byte r_xPin, byte t_xPin, bool inverse = false);
	// конструктор
	ArduinoModBusRTU_Master(byte timeOutTransmit, byte timeOutRecieve, byte directPin, byte r_xPin, byte t_xPin, bool inverse = false);
	

	void update();  // загрузка данных
	void read(byte adress, unsigned int* reg, unsigned int holdingRegBegin, unsigned int holdingRegNumber); // чтение регистров хранения
	void writeSingle(byte adress, unsigned int data, unsigned int holdingRegBegin); // запись одного регистра хранения
	void writeMultiple(byte adress, unsigned int* reg, unsigned int holdingRegBegin, unsigned int holdingRegNumber); // запись нескольких регистров хранения	
	byte state; // состояние обмена 

private:
	byte _adress; // адрес       
	byte _timeOutTransmit; // время тайм-аута передачи данных в циклах вызова update()
	byte _timeOutRecieve; // время тайм-аута приема данных в циклах вызова update()
	unsigned int * _reg;  // указатель на регистр данных
	byte _directPin;  // вывод направления передачи 
	boolean _noDirectPin;  // признак нет вывода направление 

	unsigned int calculateCRC(byte buffSize); // расчет контрольной суммы 
	void errorFrame(byte st); // ошибка кадра

	byte _frameBuffer[64]; // буфер кадра
	byte _dataNum;  // число байтов  
	byte _mode; // режим
	byte _timeOutTrCounter; // счетчик времени тайм-аута передачи
	byte _timeOutRcCounter; // счетчик времени тайм-аута приема
	unsigned int _holdingRegBegin;  // начальный адрес регистров хранения
	unsigned int _holdingRegNumber; // количество регистров хранения
	unsigned int _data;
	
	SoftwareSerial *_serial;
};

#endif