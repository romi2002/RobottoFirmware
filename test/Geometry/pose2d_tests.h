//
// Created by abiel on 2/1/21.
//

#ifndef TEENSYROSCONTROLLER_POSE2D_TESTS_H
#define TEENSYROSCONTROLLER_POSE2D_TESTS_H

#include "Geometry/Pose2D.h"
#include <Arduino.h>
#include <cmath>

static constexpr float ep = 0.0001;

void pose2d_rotation_test(){
    Pose2D current(1, 1, 0);
    current = current.rotateBy(Rotation2D(2.0 * M_PI));

    //2pi (should still be 1, 1)
    SerialUSB.println("Rotating by 2pi");
    TEST_ASSERT_FLOAT_WITHIN(ep, 1, current.x);
    TEST_ASSERT_FLOAT_WITHIN(ep, 1, current.y);

    //pi rots
    SerialUSB.println("Rotating [1,0] by pi");
    current = Pose2D(1,0,0);
    current = current.rotateBy(Rotation2D(M_PI));
    SerialUSB.print(current.x); SerialUSB.print(','); SerialUSB.println(current.y);
    TEST_ASSERT_FLOAT_WITHIN(ep, -1, current.x);
    TEST_ASSERT_FLOAT_WITHIN(ep, 0, current.y);
    current = current.rotateBy(Rotation2D(-M_PI));
    TEST_ASSERT_FLOAT_WITHIN(ep, 1, current.x);
    TEST_ASSERT_FLOAT_WITHIN(ep, 0, current.y);

    SerialUSB.println("Rotating [0,1] by pi");
    current = Pose2D(0,1,0);
    current = current.rotateBy(Rotation2D(M_PI));
    TEST_ASSERT_FLOAT_WITHIN(ep, 0, current.x);
    TEST_ASSERT_FLOAT_WITHIN(ep, -1, current.y);
    current = current.rotateBy(Rotation2D(-M_PI));
    TEST_ASSERT_FLOAT_WITHIN(ep, 0, current.x);
    TEST_ASSERT_FLOAT_WITHIN(ep, 1, current.y);

    SerialUSB.println("Rotating [1,1] by pi");
    current = Pose2D(1,1,0);
    current = current.rotateBy(Rotation2D(M_PI));
    TEST_ASSERT_FLOAT_WITHIN(ep, -1, current.x);
    TEST_ASSERT_FLOAT_WITHIN(ep, -1, current.y);
    current = current.rotateBy(Rotation2D(-M_PI));
    TEST_ASSERT_FLOAT_WITHIN(ep, 1, current.x);
    TEST_ASSERT_FLOAT_WITHIN(ep, 1, current.y);

    //pi / 2 rot
    SerialUSB.println("Rotating [1,1] by pi / 2");
    current = Pose2D(1,1,0);
    current = current.rotateBy(Rotation2D(M_PI/2));
    SerialUSB.print(current.x); SerialUSB.print(','); SerialUSB.println(current.y);
    TEST_ASSERT_FLOAT_WITHIN(ep, -1, current.x);
    TEST_ASSERT_FLOAT_WITHIN(ep, 1, current.y);
    current = current.rotateBy(Rotation2D(-M_PI/2));
    TEST_ASSERT_FLOAT_WITHIN(ep, 1, current.x);
    TEST_ASSERT_FLOAT_WITHIN(ep, 1, current.y);
}

#endif //TEENSYROSCONTROLLER_POSE2D_TESTS_H
