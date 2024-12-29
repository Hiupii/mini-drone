#include "unity.h"
#include "../include/control.h"

void test_control_value(void)
{
  control_set_point(1, 0, 1, 0, 1, 0, 0);
  TEST_ASSERT_EQUAL(pitch_setpoint, 10);
  TEST_ASSERT_EQUAL(roll_setpoint, 10);
}

void main(void)
{
  UNITY_BEGIN();
  RUN_TEST(test_control_value);
  return UNITY_END();
}