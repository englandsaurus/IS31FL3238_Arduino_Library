/******************************************************************************
SparkFun_IS31FL3238.h
IS31FL3238 Arduino Library
Andy England @ SparkFun Electronics
March 3, 2020
https://github.com/sparkfun/Qwiic_RTC

Resources:
Uses Wire.h for i2c operation
Uses SPI.h for SPI operation

Development environment specifics:
Arduino IDE 1.6.4

This code is released under the [MIT License](http://opensource.org/licenses/MIT).
Please review the LICENSE.md file included with this example. If you have any questions 
or concerns with licensing, please contact techsupport@sparkfun.com.
Distributed as-is; no warranty is given.
******************************************************************************/

#pragma once

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <Wire.h>

//The 7-bit I2C address of the IS31FL3238
#define IS31FL3238_ADDR						0x34

//Register names:
#define CTRL 								0x00 //defines software shutdown, PWM resolution, and Oscillator clock frequency
#define CURRENT_CONTROL						0x6E
#define CH1_BASE							0x01
#define CH2_BASE							0x05
#define CH3_BASE							0x09
#define CH4_BASE							0x0D
#define CH5_BASE							0x11
#define CH6_BASE							0x15
#define CH7_BASE							0x19
#define CH8_BASE							0x1D
#define CH9_BASE							0x21
#define CH10_BASE							0x25
#define CH11_BASE							0x29
#define CH12_BASE							0x2D
#define CH13_BASE							0x31
#define CH14_BASE							0x35
#define CH15_BASE							0x39
#define CH16_BASE							0x3D
#define CH17_BASE							0x41
#define CH18_BASE							0x45
#define UPDATE_DATA							0x49
#define CHANNEL_SCALE_BASE					0x4A

#define ENABLE								true
#define DISABLE								false

#define TOTAL_CHANNELS 						18

class IS31FL3238
{
  public:
	
    IS31FL3238( void );

    bool begin(TwoWire &wirePort = Wire);
	
	void setChannel(uint8_t channelNumber, uint8_t value);
	void setAllChannels(uint8_t * values);
	void setGlobalCurrent(uint8_t value); //set's the global current control. each channel is scaled by this factor
	void setChannelCurrent(uint8_t channel, uint16_t value); //set each individual channel's scaling factor
	void setCurrentAllChannels(uint16_t value); //Shortcut to set scaling factors for all channels
	void updateChannels();

	//void enable();
	//void disable();
	
	bool readBit(uint8_t regAddr, uint8_t bitAddr);
	uint8_t readTwoBits(uint8_t regAddr, uint8_t bitAddr);
	bool writeBit(uint8_t regAddr, uint8_t bitAddr, bool bitToWrite);
	bool writeBit(uint8_t regAddr, uint8_t bitAddr, uint8_t bitToWrite);
    uint8_t readRegister(uint8_t addr);
    bool writeRegister(uint8_t addr, uint8_t val);
	bool readMultipleRegisters(uint8_t addr, uint8_t * dest, uint8_t len);
	bool writeMultipleRegisters(uint8_t addr, uint8_t * values, uint8_t len);

private:
	TwoWire *_i2cPort;
};
