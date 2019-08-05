/*
INA260.cpp - Class file for the INA260 Bi-directional Current/Power Monitor Arduino Library.

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


#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "i2c.h"

#include "INA260.h"

bool INA260::begin(uint8_t address)
{

    inaAddress = address;
    return true;
}

bool INA260::configure(INA260_averages_t avg, INA260_busConvTime_t busConvTime, INA260_shuntConvTime_t shuntConvTime, INA260_mode_t mode)
{
    uint16_t config = 0;

    config |= (avg << 9 | busConvTime << 6 | shuntConvTime << 3 | mode);

    vBusMax = 36; //V
	currentMax = 15; //A //max continuous current possible to safely run through INA260 is 15A
    currentLSB = 0.00125f; //1.25mA
	vBusLSB = 0.00125f; //1.25mV
	powerLSB = 0.01f;	//10mW

    writeRegister16(INA260_REG_CONFIG, config);

	cal_shuntCurrent_offset = 0;
	cal_vBus_offset = 0;

    return true;
}

float INA260::getMaxPossibleCurrent(void)
{
	
    return currentMax;
}

float INA260::getMaxCurrent(void)
{
    float maxCurrent = (currentLSB * 32767);
    float maxPossible = getMaxPossibleCurrent();

    if (maxCurrent > maxPossible)
    {
        return maxPossible;
    } else
    {
        return maxCurrent;
    }
}



float INA260::getMaxPower(void)
{
    return (getMaxCurrent() * vBusMax);
}


void INA260::calibrate(float shuntCurrent_offset, float vBus_offset)
{
	cal_shuntCurrent_offset = shuntCurrent_offset;
	cal_vBus_offset = vBus_offset;
}

float INA260::readBusPower(void)
{
    return (readRegister16(INA260_REG_POWER) * powerLSB);
}

float INA260::readShuntCurrent(void)
{
    return (readRegister16(INA260_REG_CURRENT) * currentLSB) + cal_shuntCurrent_offset;
}


float INA260::readBusVoltage(void)
{
    return (readRegister16(INA260_REG_BUSVOLTAGE) * vBusLSB) + cal_vBus_offset;   
}

INA260_averages_t INA260::getAverages(void)
{
    uint16_t value;

    value = readRegister16(INA260_REG_CONFIG);
    value &= 0b0000111000000000;
    value >>= 9;

    return (INA260_averages_t)value;
}

INA260_busConvTime_t INA260::getBusConversionTime(void)
{
    uint16_t value;

    value = readRegister16(INA260_REG_CONFIG);
    value &= 0b0000000111000000;
    value >>= 6;

    return (INA260_busConvTime_t)value;
}

INA260_shuntConvTime_t INA260::getShuntConversionTime(void)
{
    uint16_t value;

    value = readRegister16(INA260_REG_CONFIG);
    value &= 0b0000000000111000;
    value >>= 3;

    return (INA260_shuntConvTime_t)value;
}

INA260_mode_t INA260::getMode(void)
{
    uint16_t value;

    value = readRegister16(INA260_REG_CONFIG);
    value &= 0b0000000000000111;

    return (INA260_mode_t)value;
}

void INA260::setMaskEnable(uint16_t mask)
{
    writeRegister16(INA260_REG_MASKENABLE, mask);
}

uint16_t INA260::getMaskEnable(void)
{
    return readRegister16(INA260_REG_MASKENABLE);
}

void INA260::enableOverCurrentLimitAlert(void)
{
    writeRegister16(INA260_REG_MASKENABLE, INA260_BIT_OCL);
}

void INA260::enableUnderCurrentLimitAlert(void)
{
    writeRegister16(INA260_REG_MASKENABLE, INA260_BIT_UCL);
}

void INA260::enableBusOvertLimitAlert(void)
{
    writeRegister16(INA260_REG_MASKENABLE, INA260_BIT_BOL);
}

void INA260::enableBusUnderLimitAlert(void)
{
    writeRegister16(INA260_REG_MASKENABLE, INA260_BIT_BUL);
}

void INA260::enableOverPowerLimitAlert(void)
{
    writeRegister16(INA260_REG_MASKENABLE, INA260_BIT_POL);
}

void INA260::enableConversionReadyAlert(void)
{
    writeRegister16(INA260_REG_MASKENABLE, INA260_BIT_CNVR);
}

void INA260::setBusVoltageLimit(float voltage)
{
    uint16_t value = (voltage - cal_vBus_offset) / vBusLSB;
    writeRegister16(INA260_REG_ALERTLIMIT, value);
}

void INA260::setCurrentLimit(float current)
{
	uint16_t value = (current - cal_shuntCurrent_offset) / currentLSB;
    writeRegister16(INA260_REG_ALERTLIMIT, value);
}

void INA260::setPowerLimit(float watts)
{
    uint16_t value = watts / powerLSB;
    writeRegister16(INA260_REG_ALERTLIMIT, value);
}

void INA260::setAlertInvertedPolarity(bool inverted)
{
    uint16_t temp = getMaskEnable();

    if (inverted)
    {
        temp |= INA260_BIT_APOL;
    } else
    {
        temp &= ~INA260_BIT_APOL;
    }

    setMaskEnable(temp);
}

void INA260::setAlertLatch(bool latch)
{
    uint16_t temp = getMaskEnable();

    if (latch)
    {
        temp |= INA260_BIT_LEN;
    } else
    {
        temp &= ~INA260_BIT_LEN;
    }

    setMaskEnable(temp);
}

bool INA260::isMathOverflow(void)
{
    return ((getMaskEnable() & INA260_BIT_OVF) == INA260_BIT_OVF);
}

bool INA260::isAlert(void)
{
    return ((getMaskEnable() & INA260_BIT_AFF) == INA260_BIT_AFF);
}

int16_t INA260::readRegister16(uint8_t reg)
{
	int16_t value;
    
	uint8_t val[2]={0};

    i2c_read(inaAddress,reg,val,2);

	value = val[0] << 8 | val[1];
    return value;
}

void INA260::writeRegister16(uint8_t reg, uint16_t val)
{
    
	uint8_t value[2]={0};
	value[1] = (uint8_t)val;
	value[0] = val>>8;
	
	i2c_write(inaAddress,reg,value,2);
	
}