/*
Библиотека разработана Калининым Эдуардом
mypractic.ru
*/

#include "Arduino.h"
#include "ArduinoModBusRTU_Slave.h"

// ------------------------------ конструктор -------------------------------
ArduinoModBusRTU_Slave::ArduinoModBusRTU_Slave(byte adress,byte timeOut,unsigned int * holdingRegTable,unsigned int lengthTable) {

	_adress = adress;
	_timeOut = timeOut;
	_holdingRegTable = holdingRegTable;
	_lengthTable = lengthTable;
	_noDirectPin = true;
	_mode = 0;
}

ArduinoModBusRTU_Slave::ArduinoModBusRTU_Slave(byte adress, byte timeOut, unsigned int * holdingRegTable, unsigned int lengthTable, byte directPin) {

	_adress = adress;
	_timeOut = timeOut;
	_holdingRegTable = holdingRegTable;
	_lengthTable = lengthTable;
	_directPin = directPin;

	// вывод направления
	_noDirectPin = false;
	pinMode(_directPin, OUTPUT);
	digitalWrite(_directPin, LOW);
	_mode = 0;
}

// ------------------------ проверка и перезагрузка данных ---------------------------
void ArduinoModBusRTU_Slave::update() {
	_timeOutCounter++;

	if (_mode == 0) {
		// ожидание данных порта
		if (Serial.available() != 0) {
			_dataNum = 0;
			_timeOutCounter = 0;
			_mode = 1;
		}
	}

	if (_mode == 1) {
		// прием данных, ожидание тайм-аута
		if (Serial.available() != 0) {
			_frameBuffer[_dataNum] = Serial.read();
			_dataNum++;
			if (_dataNum >= 64) errorFrame();  // ошибка
			_timeOutCounter = 0;
		}
		if (_timeOutCounter > _timeOut) {
			// тайм-аут
			_timeOutCounter = 0;
			// расшифровка кадра

			if (_dataNum < 8) errorFrame();  // ошибка, < 8 данных не бывает
			else {
				// проверка контрольной суммы
				if (calculateCRC(_dataNum - 2) != ((((unsigned int)_frameBuffer[_dataNum - 2]) << 8)
					| ((unsigned int)_frameBuffer[_dataNum - 1]))) {

					// ошибка контрольной суммы
					errorFrame();
				}
				else {
					// контрольная сумма правильная

					// проверка адреса ведомого
					if (_frameBuffer[0] == 0) {

						if (_frameBuffer[1] == 6) {
							//--------------------------------- функция записи одного регистра (широковещательная транзакция)
							// проверка адреса регистра
							unsigned int adr = (((unsigned int)_frameBuffer[2]) << 8) | (unsigned int)_frameBuffer[3];
							if (adr < _lengthTable) {
								// адрес допустимый
								*(_holdingRegTable + adr) = (((unsigned int)_frameBuffer[4]) << 8) | (unsigned int)_frameBuffer[5];
								flagWrite = true;
							}
						}

						else if (_frameBuffer[1] == 16) {
							//---------------------------------- функция записи нескольких регистров (широковещательная транзакция)
							// проверка адреса регистра
							unsigned int adr = (((unsigned int)_frameBuffer[2]) << 8) | (unsigned int)_frameBuffer[3];
							if ((adr + ((((unsigned int)_frameBuffer[4]) << 8) | (unsigned int)_frameBuffer[5])) <= _lengthTable) {
								// адрес допустимый
								unsigned int num = (((unsigned int)_frameBuffer[4]) << 8) | (unsigned int)_frameBuffer[5];
								for (unsigned int i = 0; i<num; i++) {
									*(_holdingRegTable + adr) = (((unsigned int)_frameBuffer[i * 2 + 7]) << 8) | (unsigned int)_frameBuffer[i * 2 + 8];
									adr++;
								}
								flagWrite = true;
							}
						}
						errorFrame();
					}

					else if (_frameBuffer[0] == _adress) {
						// адрес ведомого совпал
						// проверка функции

						if (_frameBuffer[1] == 3) {
							//--------------------------------- функция чтения регистров
							// проверка адреса регистра
							unsigned int adr = (((unsigned int)_frameBuffer[2]) << 8) | (unsigned int)_frameBuffer[3];
							unsigned int num = (((unsigned int)_frameBuffer[4]) << 8) | (unsigned int)_frameBuffer[5];
							if ((adr + num) <= _lengthTable) {
								// адрес допустимый

								// ответ
								_frameBuffer[2] = num * 2;
								for (unsigned int i = 0; i<num; i++) {
									_frameBuffer[i * 2 + 3] = (byte)((*(_holdingRegTable + adr)) >> 8);
									_frameBuffer[i * 2 + 4] = (byte)(*(_holdingRegTable + adr));
									adr++;
								}
								flagRead = true;
								unsigned int crc = calculateCRC(3 + num * 2);

								_frameBuffer[3 + num * 2] = (byte)(crc >> 8);
								_frameBuffer[4 + num * 2] = (byte)(crc);
								_dataNum = 5 + num * 2;
								_mode = 2; // переход на ответ                                                                                              
							}
							else {
								// ошибка адреса регистра (02)
								_frameBuffer[1] |= 0x80;
								_frameBuffer[2] = 2;
								unsigned int crc = calculateCRC(3);
								_frameBuffer[3] = (byte)(crc >> 8);
								_frameBuffer[4] = (byte)(crc);
								_dataNum = 5;
								_mode = 2; // переход на ответ     
							}
						}

						else if (_frameBuffer[1] == 6) {
							//--------------------------------- функция записи одного регистра
							// проверка адреса регистра
							unsigned int adr = (((unsigned int)_frameBuffer[2]) << 8) | (unsigned int)_frameBuffer[3];
							if (adr < _lengthTable) {
								// адрес допустимый
								*(_holdingRegTable + adr) = (((unsigned int)_frameBuffer[4]) << 8) | (unsigned int)_frameBuffer[5];
								flagWrite = true;
								_mode = 2; // переход на ответ                                             
							}
							else {
								// ошибка адреса регистра (02)
								_frameBuffer[1] |= 0x80;
								_frameBuffer[2] = 2;
								unsigned int crc = calculateCRC(3);
								_frameBuffer[3] = (byte)(crc >> 8);
								_frameBuffer[4] = (byte)(crc);
								_dataNum = 5;
								_mode = 2; // переход на ответ      
							}
						}

						else if (_frameBuffer[1] == 16) {
							//---------------------------------- функция записи нескольких регистров
							// проверка адреса регистра
							unsigned int adr = (((unsigned int)_frameBuffer[2]) << 8) | (unsigned int)_frameBuffer[3];
							if ((adr + ((((unsigned int)_frameBuffer[4]) << 8) | (unsigned int)_frameBuffer[5])) <= _lengthTable) {
								// адрес допустимый
								unsigned int num = (((unsigned int)_frameBuffer[4]) << 8) | (unsigned int)_frameBuffer[5];
								for (unsigned int i = 0; i<num; i++) {
									*(_holdingRegTable + adr) = (((unsigned int)_frameBuffer[i * 2 + 7]) << 8) | (unsigned int)_frameBuffer[i * 2 + 8];
									adr++;
								}
								flagWrite = true;
								// ответ
								unsigned int crc = calculateCRC(6);
								_frameBuffer[6] = (byte)(crc >> 8);
								_frameBuffer[7] = (byte)(crc);
								_dataNum = 8;
								_mode = 2; // переход на ответ                                                                                              
							}
							else {
								// ошибка адреса регистра (02)
								_frameBuffer[1] |= 0x80;
								_frameBuffer[2] = 2;
								unsigned int crc = calculateCRC(3);
								_frameBuffer[3] = (byte)(crc >> 8);
								_frameBuffer[4] = (byte)(crc);
								_dataNum = 5;
								_mode = 2; // переход на ответ                                                                                              
							}
						}
						else {
							// ошибка кода функции
							// ошибка кода функции (01)
							_frameBuffer[1] |= 0x80;
							_frameBuffer[2] = 1;
							unsigned int crc = calculateCRC(3);
							_frameBuffer[3] = (byte)(crc >> 8);
							_frameBuffer[4] = (byte)(crc);
							_dataNum = 5;
							_mode = 2; // переход на ответ                                                                                              
						}
					}
					else errorFrame();  // адрес ведомого не совпал          
				}
			}
		}
	}

	if (_mode == 2) {
		// пауза перед ответом
		if (_timeOutCounter > _timeOut) {
			// ответ
			if (_noDirectPin == false) digitalWrite(_directPin, HIGH);
			for (byte i = 0; i<_dataNum; i++) Serial.write(_frameBuffer[i]);
			_mode = 3; // переход на ожидание окончания передачи
		}
	}

	if (_mode == 3) {
		// ожидание окончания передачи    
		if ((UCSR0A & 64) != 0) {
			if (_noDirectPin == false) digitalWrite(_directPin, LOW);
			errorFrame();
		}
	}
}

//--------------------------- ошибка кадра -------------------------
void ArduinoModBusRTU_Slave::errorFrame() {
	while (true) { if (Serial.read() == 0xffff) break; } // сброс порта
	_mode = 0;
}

//------------------------- расчет контрольной суммы ---------------------------
unsigned int ArduinoModBusRTU_Slave::calculateCRC(byte buffSize) {
	unsigned int temp1, temp2, sighn;
	temp1 = 0xffff;
	for (byte i = 0; i < buffSize; i++) {
		temp1 = temp1 ^ _frameBuffer[i];
		for (byte j = 1; j <= 8; j++) {
			sighn = temp1 & 0x0001;
			temp1 >>= 1;
			if (sighn)
				temp1 ^= 0xA001;
		}
	}
	temp2 = temp1 >> 8;
	temp1 = (temp1 << 8) | temp2;
	temp1 &= 0xffff;
	return temp1;
}

// ------------------------ установка адреса ---------------------------
void ArduinoModBusRTU_Slave::setAdress(byte adress) {
	_adress = adress;
}
