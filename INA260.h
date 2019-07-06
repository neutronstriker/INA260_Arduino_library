/*
INA260.h - Header file for the Bi-directional Current/Power Monitor Arduino Library.

Version: 1.0.0
Date: 02-July-2019
(c) 2019 N.Srinivas (a.k.a. neutronstriker / www.xmechatronics.com)
based on INA226 Arduino Lib by Korneliusz Jarzebski (www.jarzebski.pl)

This program is free software: you can redistribute it and/or modify
it under the terms of the version 3 GNU General Public License as
published by the Free Software Foundation.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef INA260_h
#define INA260_h

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define INA260_ADDRESS              (0x40)

#define INA260_REG_CONFIG           (0x00)
#define INA260_REG_CURRENT          (0x01)
#define INA260_REG_BUSVOLTAGE       (0x02)
#define INA260_REG_POWER            (0x03)
#define INA260_REG_MASKENABLE       (0x06)
#define INA260_REG_ALERTLIMIT       (0x07)

#define INA260_BIT_OCL              (0x8000)
#define INA260_BIT_UCL              (0x4000)
#define INA260_BIT_BOL              (0x2000)
#define INA260_BIT_BUL              (0x1000)
#define INA260_BIT_POL              (0x0800)
#define INA260_BIT_CNVR             (0x0400)
#define INA260_BIT_AFF              (0x0010)
#define INA260_BIT_CVRF             (0x0008)
#define INA260_BIT_OVF              (0x0004)
#define INA260_BIT_APOL             (0x0002)
#define INA260_BIT_LEN              (0x0001)

typedef enum
{
    INA260_AVERAGES_1             = 0b000,
    INA260_AVERAGES_4             = 0b001,
    INA260_AVERAGES_16            = 0b010,
    INA260_AVERAGES_64            = 0b011,
    INA260_AVERAGES_128           = 0b100,
    INA260_AVERAGES_256           = 0b101,
    INA260_AVERAGES_512           = 0b110,
    INA260_AVERAGES_1024          = 0b111
} INA260_averages_t;

typedef enum
{
    INA260_BUS_CONV_TIME_140US    = 0b000,
    INA260_BUS_CONV_TIME_204US    = 0b001,
    INA260_BUS_CONV_TIME_332US    = 0b010,
    INA260_BUS_CONV_TIME_588US    = 0b011,
    INA260_BUS_CONV_TIME_1100US   = 0b100,
    INA260_BUS_CONV_TIME_2116US   = 0b101,
    INA260_BUS_CONV_TIME_4156US   = 0b110,
    INA260_BUS_CONV_TIME_8244US   = 0b111
} INA260_busConvTime_t;

typedef enum
{
    INA260_SHUNT_CONV_TIME_140US   = 0b000,
    INA260_SHUNT_CONV_TIME_204US   = 0b001,
    INA260_SHUNT_CONV_TIME_332US   = 0b010,
    INA260_SHUNT_CONV_TIME_588US   = 0b011,
    INA260_SHUNT_CONV_TIME_1100US  = 0b100,
    INA260_SHUNT_CONV_TIME_2116US  = 0b101,
    INA260_SHUNT_CONV_TIME_4156US  = 0b110,
    INA260_SHUNT_CONV_TIME_8244US  = 0b111
} INA260_shuntConvTime_t;

typedef enum
{
    INA260_MODE_POWER_DOWN      = 0b000,
    INA260_MODE_SHUNT_TRIG      = 0b001,
    INA260_MODE_BUS_TRIG        = 0b010,
    INA260_MODE_SHUNT_BUS_TRIG  = 0b011,
    INA260_MODE_ADC_OFF         = 0b100,
    INA260_MODE_SHUNT_CONT      = 0b101,
    INA260_MODE_BUS_CONT        = 0b110,
    INA260_MODE_SHUNT_BUS_CONT  = 0b111,
} INA260_mode_t;

class INA260
{
    public:

	bool begin(uint8_t address = INA260_ADDRESS);
	bool configure(INA260_averages_t avg = INA260_AVERAGES_1, INA260_busConvTime_t busConvTime = INA260_BUS_CONV_TIME_1100US, INA260_shuntConvTime_t shuntConvTime = INA260_SHUNT_CONV_TIME_1100US, INA260_mode_t mode = INA260_MODE_SHUNT_BUS_CONT);

	INA260_averages_t getAverages(void);
	INA260_busConvTime_t getBusConversionTime(void);
	INA260_shuntConvTime_t getShuntConversionTime(void);
	INA260_mode_t getMode(void);

	void enableOverCurrentLimitAlert(void);
	void enableUnderCurrentLimitAlert(void);
	void enableBusOvertLimitAlert(void);
	void enableBusUnderLimitAlert(void);
	void enableOverPowerLimitAlert(void);
	void enableConversionReadyAlert(void);

	void setBusVoltageLimit(float voltage);
	void setCurrentLimit(float voltage);
	void setPowerLimit(float watts);

	void setAlertInvertedPolarity(bool inverted);
	void setAlertLatch(bool latch);

	bool isMathOverflow(void);
	bool isAlert(void);

	float readShuntCurrent(void);
	float readBusPower(void);
	float readBusVoltage(void);

	float getMaxPossibleCurrent(void);
	float getMaxCurrent(void);
	float getMaxPower(void);


	void setMaskEnable(uint16_t mask);
    private:

	int8_t inaAddress;
	float currentLSB, powerLSB, vBusLSB;
	float currentMax, vBusMax;

	
	uint16_t getMaskEnable(void);

	void writeRegister16(uint8_t reg, uint16_t val);
	int16_t readRegister16(uint8_t reg);
};

#endif