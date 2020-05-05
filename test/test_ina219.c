#include "test_ina219.h"

struct ina219_dev_s dev =
{
  .slave_addr = 0x41,
  .read = i2c_read,
  .write = i2c_write,
  .delay_us = delay_us,
};

struct ina219_field_data_s data =
{
  .current = 0,
  .shunt_voltage = 0,
  .bus_voltage = 0,
  .power = 0,
};

static void check_reset(void)
{
  expect_value(__wrap_i2c_read, dev_id, dev.slave_addr);
  expect_value(__wrap_i2c_read, reg_addr, INA219_REG_ADDR_CONFIGURATION);
  will_return(__wrap_i2c_read, 0x39); /* MSB*/
  will_return(__wrap_i2c_read, 0x9F); /* LSB */
  expect_value(__wrap_i2c_read, len, 2); /* len */
  will_return(__wrap_i2c_read, 0); /* return */

  expect_value(__wrap_i2c_write, dev_id, dev.slave_addr);
  expect_value(__wrap_i2c_write, reg_addr, INA219_REG_ADDR_CONFIGURATION);
  uint8_t ptr[] = {0xB9, 0x9F};
  expect_memory(__wrap_i2c_write, reg_data, ptr, 2);
  expect_value(__wrap_i2c_write, len, 2); /* len */
  will_return(__wrap_i2c_write, 0); /* return */
}

static void check_get_configuration(void)
{
  expect_value(__wrap_i2c_read, dev_id, dev.slave_addr);
  expect_value(__wrap_i2c_read, reg_addr, INA219_REG_ADDR_CONFIGURATION);
  will_return(__wrap_i2c_read, 0x39); /* MSB*/
  will_return(__wrap_i2c_read, 0x9F); /* LSB */
  expect_value(__wrap_i2c_read, len, 2); /* len */
  will_return(__wrap_i2c_read, 0); /* return */
}

static void check_set_calib(const struct ina219_params_s *params)
{
  uint16_t reg_value = (4096 << 1);
  uint8_t lsb = ((reg_value & 0x00FF) >> 0);
  uint8_t msb = ((reg_value & 0xFF00) >> 8);

  expect_value(__wrap_i2c_write, dev_id, dev.slave_addr);
  expect_value(__wrap_i2c_write, reg_addr, INA219_REG_ADDR_CALIBRATION);
  uint8_t ptr[] = {msb, lsb};
  expect_memory(__wrap_i2c_write, reg_data, ptr, 2);
  expect_value(__wrap_i2c_write, len, 2); /* len */
  will_return(__wrap_i2c_write, 0); /* return */
}

static void check_set_config(const struct ina219_params_s *params)
{
  uint16_t reg_value = 0;
  reg_value = (((uint16_t)(params->sensor_mode) << 0 )            |
              ((uint16_t)(params->shunt_voltage_resolution) << 3) |
              ((uint16_t)(params->bus_voltage_resolution) << 7)   |
              ((uint16_t)(params->shunt_voltage_range) << 11)     |
              ((uint16_t)(params->bus_voltage_range) << 13));

  uint8_t lsb = ((reg_value & 0x00FF) >> 0);
  uint8_t msb = ((reg_value & 0xFF00) >> 8);

  expect_value(__wrap_i2c_read, dev_id, dev.slave_addr);
  expect_value(__wrap_i2c_read, reg_addr, INA219_REG_ADDR_CONFIGURATION);
  will_return(__wrap_i2c_read, 0x39); /* MSB*/
  will_return(__wrap_i2c_read, 0x9F); /* LSB */
  expect_value(__wrap_i2c_read, len, 2); /* len */
  will_return(__wrap_i2c_read, 0); /* return */

  expect_value(__wrap_i2c_write, dev_id, dev.slave_addr);
  expect_value(__wrap_i2c_write, reg_addr, INA219_REG_ADDR_CONFIGURATION);
  uint8_t ptr[] = {msb, lsb};
  expect_memory(__wrap_i2c_write, reg_data, ptr, 2);
  expect_value(__wrap_i2c_write, len, 2); /* len */
  will_return(__wrap_i2c_write, 0); /* return */
}

static void check_math_overflow_flag(void)
{
  uint8_t ovf_bit = 0x0;
  expect_value(__wrap_i2c_read, dev_id, dev.slave_addr);
  expect_value(__wrap_i2c_read, reg_addr, INA219_REG_ADDR_BUS_VOLTAGE);
  will_return(__wrap_i2c_read, 0x00); /* MSB*/
  will_return(__wrap_i2c_read, ovf_bit); /* LSB */
  expect_value(__wrap_i2c_read, len, 2); /* len */
  will_return(__wrap_i2c_read, 0); /* return */
}

static void check_get_current(void)
{
  expect_value(__wrap_i2c_read, dev_id, dev.slave_addr);
  expect_value(__wrap_i2c_read, reg_addr, INA219_REG_ADDR_CURRENT);
  will_return(__wrap_i2c_read, 0x12); /* MSB*/
  will_return(__wrap_i2c_read, 0x34); /* LSB */
  expect_value(__wrap_i2c_read, len, 2); /* len */
  will_return(__wrap_i2c_read, 0); /* return */
}

static void check_get_bus_voltage(void)
{
  expect_value(__wrap_i2c_read, dev_id, dev.slave_addr);
  expect_value(__wrap_i2c_read, reg_addr, INA219_REG_ADDR_BUS_VOLTAGE);
  will_return(__wrap_i2c_read, 0x12); /* MSB*/
  will_return(__wrap_i2c_read, 0x00); /* LSB */
  expect_value(__wrap_i2c_read, len, 2); /* len */
  will_return(__wrap_i2c_read, 0); /* return */
}

static void check_get_shunt_voltage(void)
{
  expect_value(__wrap_i2c_read, dev_id, dev.slave_addr);
  expect_value(__wrap_i2c_read, reg_addr, INA219_REG_ADDR_SHUNT_VOLTAGE);
  will_return(__wrap_i2c_read, 0x12); /* MSB*/
  will_return(__wrap_i2c_read, 0x34); /* LSB */
  expect_value(__wrap_i2c_read, len, 2); /* len */
  will_return(__wrap_i2c_read, 0); /* return */
}

static void check_get_power(void)
{
  expect_value(__wrap_i2c_read, dev_id, dev.slave_addr);
  expect_value(__wrap_i2c_read, reg_addr, INA219_REG_ADDR_POWER);
  will_return(__wrap_i2c_read, 0x12); /* MSB*/
  will_return(__wrap_i2c_read, 0x34); /* LSB */
  expect_value(__wrap_i2c_read, len, 2); /* len */
  will_return(__wrap_i2c_read, 0); /* return */
}

static void start_single_shot(enum sensor_mode_e mode)
{
  struct ina219_params_s params;
  params.sensor_mode = mode;
  uint16_t reg_value = 0x399F;
  reg_value &= ~(INA219_REG_CONFIG_MASK_MODE);
  reg_value |= mode;
  uint8_t lsb = ((reg_value & 0x00FF) >> 0);
  uint8_t msb = ((reg_value & 0xFF00) >> 8);
  uint8_t ptr[] = {msb, lsb};
  expect_value(__wrap_i2c_read, dev_id, dev.slave_addr);
  expect_value(__wrap_i2c_read, reg_addr, INA219_REG_ADDR_CONFIGURATION);
  will_return(__wrap_i2c_read, msb); /* MSB*/
  will_return(__wrap_i2c_read, lsb); /* LSB */
  expect_value(__wrap_i2c_read, len, 2); /* len */
  will_return(__wrap_i2c_read, 0); /* return */

  expect_value(__wrap_i2c_write, dev_id, dev.slave_addr);
  expect_value(__wrap_i2c_write, reg_addr, INA219_REG_ADDR_CONFIGURATION);

  reg_value = 0x399F;
  reg_value &= ~(INA219_REG_CONFIG_MASK_MODE);
  if((mode != SHUNT_BUS_TRIGGERED) &&
     (mode != BUS_TRIGGERED)       &&
     (mode != SHUNT_TRIGGERED))
  {
    reg_value |= SHUNT_BUS_TRIGGERED;
  }
  else
  {
    reg_value |= mode;
  }

  lsb = ((reg_value & 0x00FF) >> 0);
  msb = ((reg_value & 0xFF00) >> 8);
  ptr[0] = msb;
  ptr[1] = lsb;
  expect_memory(__wrap_i2c_write, reg_data, ptr, 2);
  expect_value(__wrap_i2c_write, len, 2); /* len */
  will_return(__wrap_i2c_write, 0); /* return */

  assert_int_equal(ina219_start_single_shot(&dev, &params), INA219_OK);
}

static void test_macros(void **state)
{
  /* Coverage is 100% */
  uint16_t val = 0x1234;
  uint8_t lsb = _REG_16BITS_TO_8BITS_LSB(val);
  uint8_t msb = _REG_16BITS_TO_8BITS_MSB(val);
  val = _8BITS_TO_16BITS_REG(msb, lsb);
  assert_int_equal(msb, 0x12);
  assert_int_equal(lsb, 0x34);
  assert_int_equal(val, 0x1234);
}

static void test_init(void **state)
{
  /* Coverage is 100% */
  check_reset();
  check_get_configuration();
  assert_int_equal(ina219_init(&dev), INA219_OK);
}

static void test_config(void **state)
{
  /* Too arguments for testing all issues (2^5 tests are needed) */
  struct ina219_params_s params;

  params.calibration = 4096,
  params.sensor_mode = POWER_DOWN,
  params.bus_voltage_range = BUS_VOLTAGE_RANGE_16_V,
  params.bus_voltage_resolution = RESOLUTION_12BITS_1SAMPLE,
  params.shunt_voltage_range = SHUNT_VOLTAGE_RANGE_320_MV,
  params.shunt_voltage_resolution = RESOLUTION_12BITS_1SAMPLE,
  check_set_calib(&params);
  check_set_config(&params);
  assert_int_equal(ina219_config(&dev, &params), INA219_OK);

  params.sensor_mode = SHUNT_BUS_CONTINUOUS,
  params.bus_voltage_range = BUS_VOLTAGE_RANGE_32_V,
  params.bus_voltage_resolution = RESOLUTION_12BITS_128SAMPLES,
  params.shunt_voltage_range = SHUNT_VOLTAGE_RANGE_40_MV,
  params.shunt_voltage_resolution = RESOLUTION_12BITS_128SAMPLES,
  check_set_calib(&params);
  check_set_config(&params);
  assert_int_equal(ina219_config(&dev, &params), INA219_OK);
}

static void test_get_data(void **state)
{
  check_math_overflow_flag();
  check_get_current();
  check_get_bus_voltage();
  check_get_shunt_voltage();
  check_get_power();
  assert_int_equal(ina219_get_sensor_data(&dev, &data), INA219_OK);
  assert_int_not_equal(data.current, 0);
  assert_int_not_equal(data.power, 0);
  assert_int_not_equal(data.shunt_voltage, 0);
  assert_int_not_equal(data.bus_voltage, 0);
}

static void test_start_single_shot(void **state)
{
  /* Coverage is 100% */
  start_single_shot(POWER_DOWN);
  start_single_shot(ADC_DISABLE);
  start_single_shot(SHUNT_TRIGGERED);
  start_single_shot(BUS_TRIGGERED);
  start_single_shot(SHUNT_BUS_TRIGGERED);
  start_single_shot(SHUNT_CONTINUOUS);
  start_single_shot(BUS_CONTINUOUS);
  start_single_shot(SHUNT_BUS_CONTINUOUS);
}

static void test_wait_end_conversion(void **state)
{
  struct ina219_params_s params;
  bool measure_available = false;

  params.sensor_mode = SHUNT_BUS_TRIGGERED,
  params.bus_voltage_range = BUS_VOLTAGE_RANGE_16_V,
  params.bus_voltage_resolution = RESOLUTION_12BITS_1SAMPLE,
  params.shunt_voltage_range = SHUNT_VOLTAGE_RANGE_320_MV,
  params.shunt_voltage_resolution = RESOLUTION_12BITS_1SAMPLE,

  expect_value(__wrap_delay_us, period, (uint32_t)(INA219_ADC_12BITS_1SAMPLE_DUR_US * 2));

  uint16_t reg_value = 0;
  reg_value |= INA219_REG_BUS_VOLTAGE_BIT_CNVR;
  uint8_t lsb = ((reg_value & 0x00FF) >> 0);
  uint8_t msb = ((reg_value & 0xFF00) >> 8);

  expect_value(__wrap_i2c_read, dev_id, dev.slave_addr);
  expect_value(__wrap_i2c_read, reg_addr, INA219_REG_ADDR_BUS_VOLTAGE);
  will_return(__wrap_i2c_read, msb); /* MSB*/
  will_return(__wrap_i2c_read, lsb); /* LSB */
  expect_value(__wrap_i2c_read, len, 2); /* len */
  will_return(__wrap_i2c_read, 0); /* return */

  assert_int_equal(ina219_wait_end_conversion(&dev, &params, &measure_available), INA219_OK);
  assert_true(measure_available);
}

int main(void)
{
	const struct CMUnitTest tests[] =
  {
        cmocka_unit_test(test_macros),
        cmocka_unit_test(test_init),
        cmocka_unit_test(test_config),
        cmocka_unit_test(test_get_data),
        cmocka_unit_test(test_start_single_shot),
        cmocka_unit_test(test_wait_end_conversion),
    };

  return cmocka_run_group_tests(tests, NULL, NULL);
}
