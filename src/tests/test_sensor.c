/*
 * Copyright (c) 2023, Erich Styger
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "platform.h"
#if PL_CONFIG_USE_UNIT_TESTS
#include "unity.h"
#include "test_sensor.h"
#include "sensor.h"

void TestSensor_Test(void) {
  float val;

  /* test temperature */
  val = Sensor_GetTemperature();
  TEST_ASSERT_MESSAGE(val >= -10.0f && val <= 50.0f, "Temperature not in valid range -10..50 Degree Celsius");

  /* test humidity */
  val = Sensor_GetHumidity();
  TEST_ASSERT_MESSAGE(val >= 0.0f && val <= 100.0f, "Humidity not in valid range 0..100 %");
}
#endif /* PL_CONFIG_USE_UNIT_TESTS */