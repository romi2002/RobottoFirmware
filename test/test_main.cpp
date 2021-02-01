#include <Arduino.h>
#include <unity.h>

#include "Geometry/geometry_tests.h"

void test_true(){
    TEST_ASSERT_EQUAL(1, 1);
}

void setup(){
    SerialUSB.begin(115200);
    while(!SerialUSB);
    delay(2000);

    UNITY_BEGIN();
    RUN_TEST(test_true);
    run_geometry_tests();
    UNITY_END();
}

void loop(){
}