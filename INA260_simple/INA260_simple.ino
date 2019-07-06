
/*
INA260_simple.ino - INA260 Arduino Library example sketch.
Version: 1.0.0
Date: 06-July-2019
(c) 2019 N.Srinivas (a.k.a. neutronstriker / www.xmechatronics.com)

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

#include <INA260.h>
#include <i2c.h>

#define INA260_ADDR 0x44

INA260 ina;



void setup() {
  Serial.begin(57600);
  Serial.print("DEVICE_INITIALISED\r\n");
  i2c_init(400000);

  ina.begin(INA260_ADDR);
  ina.configure(INA260_AVERAGES_4, INA260_BUS_CONV_TIME_1100US, INA260_SHUNT_CONV_TIME_1100US, INA260_MODE_SHUNT_BUS_CONT);
  
}

void loop() {

  Serial.print("INA Current:"); Serial.print(ina.readShuntCurrent()); Serial.print(" INA Bus Voltage:"); Serial.print(ina.readBusVoltage()); Serial.print(" INA Bus Power:"); Serial.print(ina.readBusPower());
  Serial.println();
  delay(1000);
}
