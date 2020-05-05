# INA219 driver
This package  contains driver APIs for using INA219 sensor.
The **INA219** is a **current shunt** and **power monitor** with an **I2C** interface.
The device monitors both shunt **voltage drop** and **bus supply voltage**, with **programmable calibration values** . An internal multiplier, enables **direct readouts** of current in amperes.
An additional multiplying register calculates power in watts. The INA219 senses accross shunts on busses that can vary **from 0 to 26 V**.
The devices uses a single 3 to 5.5 V supply, drawing a maximum of 1 mA of supply current.
The typical application are :
- Servers
- Power Management
- Battery Chargers
- Test Equipment

## Integration details

You should create a wrapper for integrating this package or simply use APIs directly on your application.
Idea of this package is to be the most portable for embedded system.
You need to include ina219 header :
``` c
#include "ina219.h"
```

## Files information

- **ina219_defs.h** : This header file has the constants, macros and datatype declarations.
- **ina219.h** : This header file contains the declarations of the sensor driver APIs.
- **ina219.c** : This source file contains the definitions of the sensor driver APIs.

## User guide
### Pin configuration
- **IN+**: Positive differential shunt voltage. Connect to positive side of shunt resistor.
- **IN-**: Negative differential shunt voltage. Connect to negative side of shunt resistor.
- **GND**: Ground.
- **SCL**: Serial bus clock line.
- **SDA**: Serial bus data line.
- **A0**: Address pin.
- **A1**: Address pin.
 > Bus voltage is measured from **IN-** to **ground.**

### I2C bus communication
16 possibles adresses for the sensor (also call slave_addr) depend on pin A0 and A1.

| A1 | A0 | Slave_Addr |
|----|----|------------|
|GND |GND |0x40 |
|GND |Vs+ |0x41 |
|GND |SDA |0x42 |
|GND |SCL |0x43 |
|Vs+ |GND |0x44 |
|Vs+ |Vs+ |0x45 |
|Vs+ |SDA |0x46 |
|Vs+ |SCL |0x47 |
|SDA |GND |0x48 |
|SDA |Vs+ |0x49 |
|SDA |SDA |0x4A |
|SDA |SCL |0x4B |
|SCL |GND |0x4C |
|SCL |Vs+ |0x4D |
|SCL |SDA |0x4E |
|SCL |SCL |0x4F |

INA219 supports transmission protocol for fast (from 1 to 400kHz) and high-speed (from 1kHz to 2.56MHz).

### Initializing the sensor

To initialize the sensor, you will first need to create a device structure. You can do this by creating an instance of the structure ina219_dev_s. Then go on to fill in the various parameters as shown below

``` c
  struct ina219_dev_s sensor = {
    .slave_addr = 0x41;
    .read = i2c_read;
    .write = i2c_write;
    .delay_us = delay_us;
  };

  rslt = ina219_init(&sensor);
  if(rslt != INA219_OK) {
    printf("[INA219] Init sensor fail : %d\r\n", rslt);
  }
```
> ```ina219_init``` realizes a software reset and check the default value of configuration register.

### Configuring the sensor
The sensor can be set for triggering  only one measure or for running continually.
This exemple below, explain the one trigger mode :

``` c
  struct ina219_params_s params = {
    .calibration = 4096,
    .sensor_mode = POWER_DOWN,
    .bus_voltage_range = BUS_VOLTAGE_RANGE_16_V,
    .bus_voltage_resolution = RESOLUTION_12BITS_1SAMPLE,
    .shunt_voltage_range = SHUNT_VOLTAGE_RANGE_320_MV,
    .shunt_voltage_resolution = RESOLUTION_12BITS_1SAMPLE,
  };

  rslt = ina219_config(&sensor, &params);
  if(rslt != INA219_OK) {
    printf("[INA219] Config sensor fail : %d\r\n", rslt);
  }
```
> All data configuration are store on ina219.h file as **enum type**.

### Reading sensor data in trigger mode

``` c
  struct ina219_field_data_s data;
  int8_t rslt;
  bool measure_available = false;

  rslt = ina219_start_single_shot();
  if(rslt != INA219_OK){
    printf("[INA219] Error start acquisition : %d\r\n", rslt);
  }

  rslt = ina219_wait_end_conversion(&sensor, &params, &measure_available);
  if((rslt != INA219_OK) || (measure_available != true)){
    printf("[INA219] Error conversion : %d\r\n", rslt);
  }

  rslt = ina219_get_sensor_data(&data);
  if(rslt != INA219_OK){
    printf("[INA219] Error get data : %d\r\n", rslt);
  }
  else{
    printf("[INA219] Get data : Current %d uA, Shunt Voltage %d mV Bus_voltage %d mV Power %d uW\r\n",
    data.current, data.shunt_voltage, data.bus_voltage, data.power);
  }
```
> Each functions return a code value :
> - ```INA219_OK``` : success operation :)
> - ```INA219_I2C_ERROR``` : Software or hardware error for I2C bus communication.
> - ```INA219_HARD_ERROR``` : Hardware error due to configuration.
> - ```INA219_SOFT_ERROR``` : Software error due to programmers.
### Templates for function pointers

``` c
void delay_us(uint32_t period)
{
  /*
   * Return control or wait,
   * for a period amount of microseconds
   */
}
int8_t i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

    /*
     * The parameter dev_id can be used as a variable to store the I2C address of the device
     */

    /*
     * Data on the bus should be like
     * |------------+---------------------|
     * | I2C action | Data                |
     * |------------+---------------------|
     * | Start      | -                   |
     * | Write      | (reg_addr)          |
     * | Stop       | -                   |
     * | Start      | -                   |
     * | Read       | (reg_data[0])       |
     * | Read       | (....)              |
     * | Read       | (reg_data[len - 1]) |
     * | Stop       | -                   |
     * |------------+---------------------|
     */

    return rslt;
}

int8_t i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

    /*
     * The parameter dev_id can be used as a variable to store the I2C address of the device
     */

    /*
     * Data on the bus should be like
     * |------------+---------------------|
     * | I2C action | Data                |
     * |------------+---------------------|
     * | Start      | -                   |
     * | Write      | (reg_addr)          |
     * | Write      | (reg_data[0])       |
     * | Write      | (....)              |
     * | Write      | (reg_data[len - 1]) |
     * | Stop       | -                   |
     * |------------+---------------------|
     */

    return rslt;
}
```
##
