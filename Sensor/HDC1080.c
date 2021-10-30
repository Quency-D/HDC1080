/*

Arduino Library for Texas Instruments HDC1080 Digital Humidity and Temperature Sensor
Written by AA for ClosedCube
---

The MIT License (MIT)

Copyright (c) 2016-2017 ClosedCube Limited

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

*/


#include "HDC1080.h"
#include "i2c.h"
#include "usbd_cdc_if.h"
#include "math.h"
#define I2C_ADDRESS (0X40 <<1)
// extern I2C_HandleTypeDef hi2c1;
#define I2C_HANDLE hi2c1
static void writeRegister(HDC1080_Pointers reg,uint8_t *data,uint8_t size);
static void readRegister(HDC1080_Pointers reg,uint8_t *data,uint8_t size);
/**********************
HDC1080_SerialNumber readSerialNumber() {
	HDC1080_SerialNumber sernum;
	sernum.serialFirst = readData(HDC1080_SERIAL_ID_FIRST);
	sernum.serialMid = readData(HDC1080_SERIAL_ID_MID);
	sernum.serialLast = readData(HDC1080_SERIAL_ID_LAST);
	return sernum;
}
**************/
void hdc1080_init(HDC1080_MeasurementResolution humidity, HDC1080_MeasurementResolution temperature)
{
	HDC1080_Registers reg;
	reg.HumidityMeasurementResolution = 0;
	reg.TemperatureMeasurementResolution = 0;

	if (temperature == HDC1080_RESOLUTION_11BIT)
		reg.TemperatureMeasurementResolution = 0x01;

	switch (humidity)
	{
		case HDC1080_RESOLUTION_8BIT:
			reg.HumidityMeasurementResolution = 0x02;
			break;
		case HDC1080_RESOLUTION_11BIT:
			reg.HumidityMeasurementResolution = 0x01;
			break;
		default:
			break;
	}
	reg.rawData = 0x00;

	writeRegister(HDC1080_CONFIGURATION,(uint8_t *)&reg,2);

}

static void readRegister(HDC1080_Pointers reg,uint8_t *data,uint8_t size) 
{
	HAL_StatusTypeDef i2c_status =HAL_OK;
	i2c_status = HAL_I2C_Mem_Read(&I2C_HANDLE,I2C_ADDRESS,reg, 1,data, size, 1000);
	if(i2c_status != HAL_OK)
	{
		usb_printf("readRegister error\r\n");
		return ;
	}
}

static void writeRegister(HDC1080_Pointers reg,uint8_t *data,uint8_t size) 
{

	HAL_StatusTypeDef i2c_status =HAL_OK;
	i2c_status= HAL_I2C_Mem_Write(&I2C_HANDLE, I2C_ADDRESS, reg, 1, data,size,100);
	if(i2c_status != HAL_OK)
	{
		usb_printf("writeRegister error\r\n");
		return ;
	}
}

void hdc1080_read_temperature(double *temperature) 
{
	// uint16_t rawT = readData(HDC1080_TEMPERATURE);
	// return (rawT / pow(2, 16)) * 165.0 - 40.0;
	uint16_t temp =0;
	uint8_t num =0;
	uint8_t data[2] ={0};
	writeRegister(HDC1080_TEMPERATURE,&num,1);
	// HAL_I2C_Master_Transmit(&I2C_HANDLE,I2C_ADDRESS,&num,1,1000);
	HAL_Delay(30);
 	// readRegister(HDC1080_TEMPERATURE,data,2);
	HAL_I2C_Master_Receive(&I2C_HANDLE,I2C_ADDRESS,data,2,1000); 
 	temp = (data[0] << 8 | data[1]);
 	*temperature =  (temp / pow(2, 16)) * 165.0 - 40.0;
}

void hdc1080_read_humidity(double *humidity) 
{
	// uint16_t rawH = readData(HDC1080_HUMIDITY);
	// return (rawH / pow(2, 16)) * 100.0;
	uint16_t temp =0;
	uint8_t num =0;
	uint8_t data[2] ={0};
	writeRegister(HDC1080_HUMIDITY,&num,1);
	// HAL_I2C_Master_Transmit(&I2C_HANDLE,I2C_ADDRESS,&num,1,1000);
	HAL_Delay(30);
 	// readRegister(HDC1080_TEMPERATURE,data,2);
	HAL_I2C_Master_Receive(&I2C_HANDLE,I2C_ADDRESS,data,2,1000); 
 	temp = (data[0] << 8 | data[1]);
 	*temperature =  (temp / pow(2, 16)) * 100.0;
}
/**********************
void heatUp(uint8_t seconds) {
	HDC1080_Registers reg = readRegister();
	reg.Heater = 1;
	reg.ModeOfAcquisition = 1;
	writeRegister(reg);

	uint8_t buf[4];
	for (int i = 1; i < (seconds*66); i++) {
		Wire.beginTransmission(I2C_ADDRESS);
		Wire.write(0x00);
		Wire.endTransmission();
		delay(20);
		Wire.requestFrom(I2C_ADDRESS, (uint8_t)4);
		Wire.readBytes(buf, (size_t)4);
	}
	reg.Heater = 0;
	reg.ModeOfAcquisition = 0;
	writeRegister(reg);
}


double readT() {
	return readTemperature();
}

double readTemperature() {
	uint16_t rawT = readData(HDC1080_TEMPERATURE);
	return (rawT / pow(2, 16)) * 165.0 - 40.0;
}

double readH() {
	return readHumidity();
}

double readHumidity() {
	uint16_t rawH = readData(HDC1080_HUMIDITY);
	return (rawH / pow(2, 16)) * 100.0;
}
**************/
void hdc1080_read_manufacturer_id(uint16_t *id)
{
	// return readData(HDC1080_MANUFACTURER_ID);
	uint8_t data[2];
	readRegister(HDC1080_MANUFACTURER_ID,data,2);

	// usb_printf("data %X%X\r\n",data[0],data[1]);
	*id = (data[0] << 8 | data[1]);
}

/****************
uint16_t readDeviceId() {
	return readData(HDC1080_DEVICE_ID);
}

uint16_t readData(uint8_t pointer) {
	Wire.beginTransmission(I2C_ADDRESS);
	Wire.write(pointer);
	Wire.endTransmission();
	
	delay(9);
	Wire.requestFrom(I2C_ADDRESS, (uint8_t)2);

	byte msb = Wire.read();
	byte lsb = Wire.read();

	return msb << 8 | lsb;
}
*****************/

