#include <stdarg.h>
#include <stddef.h>
#include <setjmp.h>
#include <cmocka.h>

#include "test_ina219.h"

int8_t __wrap_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
  assert_non_null(reg_data);

  check_expected(dev_id);
  check_expected(reg_addr);
  for(int i=0; i < len; i++)
  {
    reg_data[i] = mock_type(uint8_t);
  }
  check_expected(len);

  return (int8_t)mock();
}

int8_t __wrap_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
  assert_non_null(reg_data);

  check_expected(dev_id);
  check_expected(reg_addr);
  check_expected(reg_data);
  check_expected(len);

  return (int8_t)mock();
}

void __wrap_delay_us(uint32_t period)
{
   check_expected(period);
}
