
#include "Arduino.h"
#include "ArduinoModBusRTU_Master.h"
#include <SoftwareSerial.h>

// ------------------------------ конструктор -------------------------------
ArduinoModBusRTU_Master::ArduinoModBusRTU_Master(byte timeOutTransmit, byte timeOutRecieve) {

	_timeOutTransmit = timeOutTransmit;
	_timeOutRecieve = timeOutRecieve;
	_noDirectPin = true;
	_mode = 0;
}

ArduinoModBusRTU_Master::ArduinoModBusRTU_Master(byte timeOutTransmit, byte timeOutRecieve, byte directPin)
{
	_timeOutTransmit = timeOutTransmit;
	_timeOutRecieve = timeOutRecieve;
	_directPin = directPin;

	// вывод направления
	_noDirectPin = false;
	pinMode(_directPin, OUTPUT);
	digitalWrite(_directPin, LOW);
	_mode = 0;
}

ArduinoModBusRTU_Master::ArduinoModBusRTU_Master(byte timeOutTransmit, byte timeOutRecieve, byte r_xPin, byte t_xPin, bool inverse = false) {

	_timeOutTransmit = timeOutTransmit;
	_timeOutRecieve = timeOutRecieve;
	_noDirectPin = true;

	_serial = new SoftwareSerial(r_xPin, t_xPin, inverse);
	_serial->begin(9600);

	_mode = 0;
}

ArduinoModBusRTU_Master::ArduinoModBusRTU_Master(byte timeOutTransmit, byte timeOutRecieve, byte directPin, byte r_xPin, byte t_xPin, bool inverse = false)
{
	_timeOutTransmit = timeOutTransmit;
	_timeOutRecieve = timeOutRecieve;
	_directPin = directPin;

	// вывод направления
	_noDirectPin = false;
	pinMode(_directPin, OUTPUT);
	digitalWrite(_directPin, LOW);

	_serial = new SoftwareSerial(r_xPin, t_xPin, inverse);
	_serial->begin(9600);

	_mode = 0;
}

// ------------------------ проверка и перезагрузка данных ---------------------------
void ArduinoModBusRTU_Master::update() {

	_timeOutTrCounter++;
	if (_timeOutTrCounter > _timeOutTransmit) _timeOutTrCounter = _timeOutTransmit;

	if (_mode == 0) {
		// ожидание команды    
	}

	else if (_mode == 1) {
		// начало операции чтение регистров хранения    
		// ожидание окончания паузы между фреймами
		if (_timeOutTrCounter >= _timeOutTransmit) {
			// формирование фрейма запроса
			if (_noDirectPin == false) digitalWrite(_directPin, HIGH);

			// адрес
			if (_serial)
				_serial->write(_adress);
			else
				Serial.write(_adress);
			_frameBuffer[0] = _adress;

			// функция
			if (_serial)
				_serial->write(3);
			else
				Serial.write(3);
			_frameBuffer[1] = 3;

			// начальный адрес
			_frameBuffer[2] = (byte)(_holdingRegBegin >> 8);
			_frameBuffer[3] = (byte)_holdingRegBegin;

			if (_serial)
			{
				_serial->write(_frameBuffer[2]);
				_serial->write(_frameBuffer[3]);
			}
			else
			{
				Serial.write(_frameBuffer[2]);
				Serial.write(_frameBuffer[3]);
			}

			// количество регистров
			_frameBuffer[4] = (byte)(_holdingRegNumber >> 8);
			_frameBuffer[5] = (byte)_holdingRegNumber;

			if (_serial)
			{
				_serial->write(_frameBuffer[4]);
				_serial->write(_frameBuffer[5]);
			}
			else
			{
				Serial.write(_frameBuffer[4]);
				Serial.write(_frameBuffer[5]);
			}
			// контрольная сумма
			unsigned int crc = calculateCRC(6);

			if (_serial)
			{
				_serial->write((byte)(crc >> 8));
				_serial->write((byte)crc);
			}
			else
			{
				Serial.write((byte)(crc >> 8));
				Serial.write((byte)crc);
			}

			_mode = 2;
		}
	}

	else if (_mode == 2) {
		// ожидание окончания передачи
		if ((UCSR0A & 64) != 0) {
			if (_noDirectPin == false) digitalWrite(_directPin, LOW);
			while (true) {
				if (_serial && _serial->read() == 0xffff || !_serial && Serial.read() == 0xffff) break;
			} // сброс порта
			_timeOutRcCounter = 0;
			_mode = 3;
		}
	}

	else if (_mode == 3) {
		// ожидание данных порта
		if (_serial && _serial->available() != 0 || !_serial && Serial.available() != 0) {
			_dataNum = 0;
			_timeOutTrCounter = 0;
			_mode = 4;
		}
		_timeOutRcCounter++;
		if (_timeOutRcCounter > _timeOutRecieve) errorFrame(2);  // ошибка тайм-аута приема          
	}

	else if (_mode == 4) {
		// прием данных

		if (_serial && _serial->available() != 0 || !_serial && Serial.available() != 0) {
			_frameBuffer[_dataNum] = _serial ? _serial->read() : Serial.read();
			_dataNum++;
			if (_dataNum >= 64) errorFrame(4);  // ошибка
			_timeOutTrCounter = 0;
		}
		if (_timeOutTrCounter >= _timeOutTransmit) {
			// тайм-аут
			_timeOutTrCounter = 0;
			// расшифровка кадра

			if (_dataNum != (5 + _holdingRegNumber * 2)) errorFrame(4);  // ошибка
			else {

				// проверка контрольной суммы
				if (calculateCRC(_dataNum - 2) != ((((unsigned int)_frameBuffer[_dataNum - 2]) << 8)
					| ((unsigned int)_frameBuffer[_dataNum - 1]))) {
					// ошибка контрольной суммы
					errorFrame(4);
				}
				else {
					// контрольная сумма правильная

					// проверка адреса
					if (_frameBuffer[0] != _adress) errorFrame(4);  // ошибка
					else {

						// проверка кода функции
						if ((_frameBuffer[1] & 0x80) != 0) {
							// ошибка 
							if (_frameBuffer[2] == 1) errorFrame(16);  // ошибка
							else if (_frameBuffer[2] == 2) errorFrame(8);  // ошибка
							else errorFrame(32);  // ошибка
						}
						else {
							if (_frameBuffer[1] != 3) errorFrame(4);  // ошибка
							else {

								// все правильно, перегрузка регистров                
								for (unsigned int i = 0; i<_holdingRegNumber; i++) {
									*(_reg + i) = (((unsigned int)_frameBuffer[i * 2 + 3]) << 8) | (unsigned int)_frameBuffer[i * 2 + 4];
								}

								_timeOutTrCounter = 0;
								errorFrame(0);  // все правильно, окончеание операции                  
							}
						}
					}
				}
			}
		}
	}

	else if (_mode == 5) {
		// начало операции запись одного регистра хранения    
		// ожидание окончания паузы между фреймами
		if (_timeOutTrCounter >= _timeOutTransmit) {
			// формирование фрейма запроса
			if (_noDirectPin == false) digitalWrite(_directPin, HIGH);

			// адрес
			if (_serial)
				_serial->write(_adress);
			else
				Serial.write(_adress);
			_frameBuffer[0] = _adress;

			// функция
			if (_serial)
				_serial->write(6);
			else
				Serial.write(6);
			_frameBuffer[1] = 6;

			// начальный адрес
			_frameBuffer[2] = (byte)(_holdingRegBegin >> 8);
			_frameBuffer[3] = (byte)_holdingRegBegin;
			if (_serial)
			{
				_serial->write(_frameBuffer[2]);
				_serial->write(_frameBuffer[3]);
			}
			else
			{
				Serial.write(_frameBuffer[2]);
				Serial.write(_frameBuffer[3]);
			}

			// данное
			_frameBuffer[4] = (byte)(_data >> 8);
			_frameBuffer[5] = (byte)_data;
			if (_serial)
			{
				_serial->write(_frameBuffer[4]);
				_serial->write(_frameBuffer[5]);
			}
			else
			{
				Serial.write(_frameBuffer[4]);
				Serial.write(_frameBuffer[5]);
			}

			// контрольная сумма
			unsigned int crc = calculateCRC(6);

			if (_serial)
			{
				_serial->write((byte)(crc >> 8));
				_serial->write((byte)crc);
			}
			else {
				Serial.write((byte)(crc >> 8));
				Serial.write((byte)crc);
			}

			_mode = 6;
		}
	}

	else if (_mode == 6) {
		// ожидание окончания передачи
		if ((UCSR0A & 64) != 0) {
			if (_noDirectPin == false) digitalWrite(_directPin, LOW);
			while (true) {
				//if (Serial.read() == 0xffff) break; 
				if (_serial && _serial->read() == 0xffff || !_serial && Serial.read() == 0xffff) break;
			} // сброс порта
			_timeOutRcCounter = 0;
			if (_adress == 0)  errorFrame(0);
			else _mode = 7;
		}
	}

	else if (_mode == 7) {
		// ожидание данных порта
		if (_serial && _serial->available() != 0 || !_serial && Serial.available() != 0) {
			_dataNum = 0;
			_timeOutTrCounter = 0;
			_mode = 8;
		}
		_timeOutRcCounter++;
		if (_timeOutRcCounter > _timeOutRecieve) errorFrame(2);  // ошибка тайм-аута приема          
	}

	else if (_mode == 8) {
		// прием данных

		//if (Serial.available() != 0) 
		if (_serial && _serial->available() != 0 || !_serial && Serial.available() != 0)
		{
			_frameBuffer[_dataNum] = _serial ? _serial->read() : Serial.read();
			_dataNum++;
			if (_dataNum >= 64) errorFrame(4);  // ошибка
			_timeOutTrCounter = 0;
		}
		if (_timeOutTrCounter >= _timeOutTransmit) {
			// тайм-аут
			_timeOutTrCounter = 0;
			// расшифровка кадра

			if (_dataNum != 8) errorFrame(4);  // ошибка
			else {

				// проверка контрольной суммы
				if (calculateCRC(6) != ((((unsigned int)_frameBuffer[6]) << 8) | ((unsigned int)_frameBuffer[7]))) {
					// ошибка контрольной суммы
					errorFrame(4);
				}
				else {
					// контрольная сумма правильная

					// проверка адреса
					if (_frameBuffer[0] != _adress) errorFrame(4);  // ошибка
					else {

						// проверка кода функции
						if ((_frameBuffer[1] & 0x80) != 0) {
							// ошибка 
							if (_frameBuffer[2] == 1) errorFrame(16);  // ошибка
							else if (_frameBuffer[2] == 2) errorFrame(8);  // ошибка
							else errorFrame(32);  // ошибка
						}
						else {
							if (_frameBuffer[1] != 6) errorFrame(4);  // ошибка
							else {

								// проверка адреса регистра
								if ((_frameBuffer[2] != ((byte)(_holdingRegBegin >> 8))) || (_frameBuffer[3] != ((byte)_holdingRegBegin))) errorFrame(4);  // ошибка
								else {

									// проверка значения регистра
									if ((_frameBuffer[4] != ((byte)(_data >> 8))) || (_frameBuffer[5] != ((byte)_data))) errorFrame(4);  // ошибка
									else {

										// все правильно
										_timeOutTrCounter = 0;
										errorFrame(0);  // все правильно, окончеание операции                                      
									}
								}
							}
						}
					}
				}
			}
		}
	}

	else if (_mode == 9) {
		// начало операции запись регистров хранения    
		// ожидание окончания паузы между фреймами
		if (_timeOutTrCounter >= _timeOutTransmit) {
			// формирование фрейма запроса
			if (_noDirectPin == false) digitalWrite(_directPin, HIGH);

			// адрес
			_frameBuffer[0] = _adress;

			// функция
			_frameBuffer[1] = 16;

			// начальный адрес
			_frameBuffer[2] = (byte)(_holdingRegBegin >> 8);
			_frameBuffer[3] = (byte)_holdingRegBegin;

			// количество регистров
			_frameBuffer[4] = (byte)(_holdingRegNumber >> 8);
			_frameBuffer[5] = (byte)_holdingRegNumber;

			// счетчик байтов
			_frameBuffer[6] = (byte)(_holdingRegNumber * 2);

			// значения регистров
			for (int i = 0; i<_holdingRegNumber; i++) {
				_frameBuffer[7 + i * 2] = (byte)((*(_reg + i)) >> 8);
				_frameBuffer[7 + i * 2 + 1] = (byte)(*(_reg + i));
			}

			for (int i = 0; i < (_holdingRegNumber * 2 + 7); i++) {
				if (_serial)
					_serial->write(_frameBuffer[i]);
				else
					Serial.write(_frameBuffer[i]);
			}

			// контрольная сумма
			unsigned int crc = calculateCRC(_holdingRegNumber * 2 + 7);
			if (_serial) {
				_serial->write((byte)(crc >> 8));
				_serial->write((byte)crc);
			}
			else {
				Serial.write((byte)(crc >> 8));
				Serial.write((byte)crc);
			}

			_mode = 10;
		}
	}

	else if (_mode == 10) {
		// ожидание окончания передачи
		if ((UCSR0A & 64) != 0) {
			if (_noDirectPin == false) digitalWrite(_directPin, LOW);
			while (true) {
				if (_serial && _serial->read() == 0xffff || !_serial && Serial.read() == 0xffff) break;
				//if (Serial.read() == 0xffff) break; 
			} // сброс порта
			_timeOutRcCounter = 0;
			if (_adress == 0)  errorFrame(0);
			else _mode = 11;
		}
	}

	else if (_mode == 11) {
		// ожидание данных порта
		if (_serial && _serial->available() != 0 || !_serial && Serial.available() != 0) {
			_dataNum = 0;
			_timeOutTrCounter = 0;
			_mode = 12;
		}
		_timeOutRcCounter++;
		if (_timeOutRcCounter > _timeOutRecieve) errorFrame(2);  // ошибка тайм-аута приема          
	}

	else if (_mode == 12) {
		// прием данных

		if (_serial && _serial->available() != 0 || !_serial && Serial.available() != 0) {
			_frameBuffer[_dataNum] = _serial ? _serial->read() : Serial.read();
			_dataNum++;
			if (_dataNum >= 64) errorFrame(4);  // ошибка
			_timeOutTrCounter = 0;
		}
		if (_timeOutTrCounter >= _timeOutTransmit) {
			// тайм-аут
			_timeOutTrCounter = 0;
			// расшифровка кадра

			if (_dataNum != 8) errorFrame(4);  // ошибка
			else {

				// проверка контрольной суммы
				if (calculateCRC(6) != ((((unsigned int)_frameBuffer[6]) << 8) | ((unsigned int)_frameBuffer[7]))) {
					// ошибка контрольной суммы
					errorFrame(4);
				}
				else {
					// контрольная сумма правильная

					// проверка адреса
					if (_frameBuffer[0] != _adress) errorFrame(4);  // ошибка
					else {

						// проверка кода функции
						if ((_frameBuffer[1] & 0x80) != 0) {
							// ошибка 
							if (_frameBuffer[2] == 1) errorFrame(16);  // ошибка
							else if (_frameBuffer[2] == 2) errorFrame(8);  // ошибка
							else errorFrame(32);  // ошибка
						}
						else {
							if (_frameBuffer[1] != 16) errorFrame(4);  // ошибка
							else {

								// проверка адреса регистра
								if ((_frameBuffer[2] != ((byte)(_holdingRegBegin >> 8))) || (_frameBuffer[3] != ((byte)_holdingRegBegin))) errorFrame(4);  // ошибка
								else {

									// проверка количества регистров
									if ((_frameBuffer[4] != ((byte)(_holdingRegNumber >> 8))) || (_frameBuffer[5] != ((byte)_holdingRegNumber))) errorFrame(4);  // ошибка
									else {

										// все правильно
										_timeOutTrCounter = 0;
										errorFrame(0);  // все правильно, окончеание операции                                      
									}
								}
							}
						}
					}
				}
			}
		}
	}

	else _mode = 0;
}

//-------------------------- чтение регистров хранения ---------------------------------
void ArduinoModBusRTU_Master::read(byte adress, unsigned int* reg, unsigned int holdingRegBegin, unsigned int holdingRegNumber) {
	state = 1;
	_adress = adress;
	_reg = reg;
	_holdingRegBegin = holdingRegBegin;
	_holdingRegNumber = holdingRegNumber;
	_mode = 1;
}

//-------------------------- запись одного регистра хранения ---------------------------------
void ArduinoModBusRTU_Master::writeSingle(byte adress, unsigned int data, unsigned int holdingRegBegin) {
	state = 1;
	_adress = adress;
	_data = data;
	_holdingRegBegin = holdingRegBegin;
	_mode = 5;
}

//-------------------------- запись регистров хранения ---------------------------------
void ArduinoModBusRTU_Master::writeMultiple(byte adress, unsigned int* reg, unsigned int holdingRegBegin, unsigned int holdingRegNumber) {
	state = 1;
	_adress = adress;
	_reg = reg;
	_holdingRegBegin = holdingRegBegin;
	_holdingRegNumber = holdingRegNumber;
	_mode = 9;
}

//------------------------- расчет контрольной суммы ---------------------------
unsigned int ArduinoModBusRTU_Master::calculateCRC(byte buffSize) {
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

//--------------------------- ошибка кадра -------------------------
void ArduinoModBusRTU_Master::errorFrame(byte st) {
	while (true) {
		if (_serial && _serial->read() == 0xffff || !_serial && Serial.read() == 0xffff) break;
		//if (Serial.read() == 0xffff) break; 
	} // сброс порта
	_mode = 0;
	state = st;
}
