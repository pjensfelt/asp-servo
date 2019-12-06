#include <math.h>

// Workspace limits
const double X_LIM[2]	= {-0.112, 2.750-0.112};// [m]
const double Y_LIM[2]	= {-0.08, 100.0}	; 		// [m] y has no upper bound -0.024
const double X_OFFSET	= 0.250; 		            // Translation from global origin to
const double Y_OFFSET	= 0.5575; 		          // arm origin

const double L_ARM_GEAR  = 0.9985-(0.036+0.017)-0.044-0.026;
const double L_ARM_WRIST = L_ARM_GEAR + 0.144; //sqrt(0.287*0.287/4 + 0.217*0.217/4); // Updated after experiment
const double D1       = 0.056 + 0.1;
const double D2       = 0.064 + 0.069;
const double THETA1   = atan(D1/L_ARM_WRIST);//0.174093309;          // arctan(d1/l_arm) computed with l_arm = 0.889m
const double THETA2   = atan(D2/L_ARM_WRIST);//0.14662888;          // arctan(d2/l_arm) computed with l_arm = 0.889m
const double L1       = sqrt(D1*D1 + L_ARM_WRIST*L_ARM_WRIST);//0.90258351414;        // sqrt(d_1^2 + arm_len^2)
const double L2       = sqrt(D2*D2 + L_ARM_WRIST*L_ARM_WRIST);//0.89860002225;        // sqrt(d_2^2 + arm_len^2)
const double TOLERANCE	= 0.00			; 		    // Dont let the arm go closer than TOLERANCE m to the geometric limit

const double PB_H_LENGTH = 0.287/2;
const double PB_H_WIDTH = 0.217/2;
const double L_PB = sqrt(PB_H_LENGTH*PB_H_LENGTH + PB_H_WIDTH*PB_H_WIDTH);
const double THETA_PB = atan(PB_H_WIDTH/PB_H_LENGTH);

const double Z_TORQUE_LIMIT = 0.00025;
