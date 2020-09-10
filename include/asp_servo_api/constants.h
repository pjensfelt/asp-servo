#include <math.h>

// Workspace limits
const double X_LIM[2] = { -0.12, 2.65 };
const double Y_LIM[2] = { -0.08, 100.0 };

// Arm constants
const double X_OFFSET = 0.250; // Translation from global origin to
const double Y_OFFSET = 0.5575; // arm origin
const double DISTANCE_A_B_AXES = 0.8768; // Distance from B axis to A axis
/**
 * Wrist plate dimensions and axis location.
 * Top view of the wrist plate
 * 
 * |- WRIST_PLATE_X_SIZE -|
 *      -----------            ---
 *      |         |             |
 *      |         |             |
 *      |    c    |    WRIST_PLATE_Y_SIZE
 *      |    o    |             |
 *      |         |             |
 *      -----------            ---
 * o marks the rotational axis of joint A, c the center of the wrist plate.
 * The position of the plate's center relative to the axis is(A_AXIS_OFFSET_X, A_AXIS_OFFSET_Y)
 * The z-axis of the frame located at o points downwards.
*/
const double A_AXIS_OFFSET_X = 0.0;
const double A_AXIS_OFFSET_Y = 0.0335;
const double WRIST_PLATE_X_SIZE = 0.125;
const double WRIST_PLATE_Y_SIZE = 0.221;

// Dont let the arm go closer than TOLERANCE m to the geometric limit
const double TOLERANCE = 0.00;

// Pressure box dimensions
const double PB_H_LENGTH = 0.287 / 2;
const double PB_H_WIDTH = 0.217 / 2;
const double L_PB = sqrt(PB_H_LENGTH * PB_H_LENGTH + PB_H_WIDTH * PB_H_WIDTH);
const double THETA_PB = atan(PB_H_WIDTH / PB_H_LENGTH);

const double Z_TORQUE_LIMIT = 0.0007;
