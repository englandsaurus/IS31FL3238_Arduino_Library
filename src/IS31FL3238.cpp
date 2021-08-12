 /******************************************************************************
SparkFun_IS31FL3238.h
IS31FL3238 Arduino Library
Andy England
April 21, 2021

Development environment specifics:
Arduino IDE 1.6.4

This code is released under the [MIT License](http://opensource.org/licenses/MIT).
Please review the LICENSE.md file included with this example. If you have any questions
or concerns with licensing, please contact techsupport@sparkfun.com.
Distributed as-is; no warranty is given.
******************************************************************************/

#include "IS31FL3238.h"

IS31FL3238::IS31FL3238( void )
{

}

bool IS31FL3238::begin(TwoWire &wirePort)
{
	_i2cPort = &wirePort;
	
	_i2cPort->beginTransmission(IS31FL3238_ADDR);
	
    if (_i2cPort->endTransmission() != 0)
	{
      return (false); //Error: Sensor did not ack
	}
	writeRegister(CTRL, 0b01000001);
	return(true);
}

void IS31FL3238::setChannel(uint8_t channelNumber, uint8_t value) //number channels at base zero (0-17)
{	
	uint8_t sendValue[4];
	sendValue[0] = value;
	sendValue[1] = 0;	
	sendValue[2] = value;
	sendValue[3] = 0;	
	
	writeMultipleRegisters((channelNumber * 4) + 1, sendValue, 4);

}

void IS31FL3238::setAllChannels(uint8_t * values)
{
	uint8_t sendValue[TOTAL_CHANNELS * 4];
	for (uint8_t channelNum = 0; channelNum < 18; channelNum++)
	{
		uint8_t base = channelNum * 4;
		sendValue[base] = values[channelNum];
		sendValue[base + 1] = 0;
		sendValue[base + 2] = values[channelNum];
		sendValue[base + 3] = 0;
	}
	writeMultipleRegisters(CH1_BASE, sendValue, TOTAL_CHANNELS * 4);
}

void IS31FL3238::setCurrentAllChannels(uint16_t value)
{
	uint8_t sendValue[36];
	for (uint8_t channel = 0; channel < 18; channel++)
	{
		sendValue[(channel * 2)] = (value & 0xF0) >> 8;
		sendValue[(channel * 2) + 1] = (value & 0x0F);	
	}
	writeMultipleRegisters(CHANNEL_SCALE_BASE, sendValue, 36);
}

void IS31FL3238::setChannelCurrent(uint8_t channel, uint16_t value)
{
	uint8_t sendValue[2];
	sendValue[0] = (value & 0xF0) >> 8;
	sendValue[1] = (value & 0x0F);
	writeMultipleRegisters(CHANNEL_SCALE_BASE + (channel * 2), sendValue, 2);
}

void IS31FL3238::updateChannels()
{
	writeRegister(UPDATE_DATA, 0x00);
}

void IS31FL3238::setGlobalCurrent(uint8_t value)
{
	writeRegister(CURRENT_CONTROL, value);
}

bool IS31FL3238::readBit(uint8_t regAddr, uint8_t bitAddr)
{
	return ((readRegister(regAddr) & (1 << bitAddr)) >> bitAddr);
}

uint8_t IS31FL3238::readTwoBits(uint8_t regAddr, uint8_t bitAddr)
{
	return ((readRegister(regAddr) & (3 << bitAddr)) >> bitAddr);
}

bool IS31FL3238::writeBit(uint8_t regAddr, uint8_t bitAddr, bool bitToWrite)
{
	uint8_t value = readRegister(regAddr);
	value &= ~(1 << bitAddr);
	value |= bitToWrite << bitAddr;
	return writeRegister(regAddr, value);
}
bool IS31FL3238::writeBit(uint8_t regAddr, uint8_t bitAddr, uint8_t bitToWrite) //If we seean unsigned eight bit, we know we have to write two bits.
{
	uint8_t value = readRegister(regAddr);
	value &= ~(3 << bitAddr);
	value |= bitToWrite << bitAddr;
	return writeRegister(regAddr, value);
}

uint8_t IS31FL3238::readRegister(uint8_t addr)
{
	_i2cPort->beginTransmission(IS31FL3238_ADDR);
	_i2cPort->write(addr);
	_i2cPort->endTransmission();

    //typecasting the 1 parameter in requestFrom so that the compiler
    //doesn't give us a warning about multiple candidates
    if (_i2cPort->requestFrom(static_cast<uint8_t>(IS31FL3238_ADDR), static_cast<uint8_t>(1)) != 0)
    {
        return _i2cPort->read();
    }
}

bool IS31FL3238::writeRegister(uint8_t addr, uint8_t val)
{
	_i2cPort->beginTransmission(IS31FL3238_ADDR);
	_i2cPort->write(addr);
	_i2cPort->write(val);
    if (_i2cPort->endTransmission() != 0)
      return (false); //Error: Sensor did not ack
	return(true);
}

bool IS31FL3238::writeMultipleRegisters(uint8_t addr, uint8_t * values, uint8_t len)
{
	_i2cPort->beginTransmission(IS31FL3238_ADDR);
	_i2cPort->write(addr);
	for (uint8_t i = 0; i < len; i++)
	{
		_i2cPort->write(values[i]);
	}

    if (_i2cPort->endTransmission() != 0)
      return (false); //Error: Sensor did not ack
	return(true);
}

bool IS31FL3238::readMultipleRegisters(uint8_t addr, uint8_t * dest, uint8_t len)
{
	_i2cPort->beginTransmission(IS31FL3238_ADDR);
	_i2cPort->write(addr);
    if (_i2cPort->endTransmission() != 0)
      return (false); //Error: Sensor did not ack

	_i2cPort->requestFrom(static_cast<uint8_t>(IS31FL3238_ADDR), len);
	for (uint8_t i = 0; i < len; i++)
	{
		dest[i] = _i2cPort->read();
	}
	
	return(true);
}
