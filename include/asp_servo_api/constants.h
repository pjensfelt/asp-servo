#include <math.h>

// Workspace limits
const double X_LIM[2]	= {-0.112, 2.750-0.112};// [m]
const double Y_LIM[2]	= {-0.024, 100.0}	; 		// [m] y has no upper bound -0.024
const double X_OFFSET	= 0.165; 		            // Translation from global origin to
const double Y_OFFSET	= 0.5525; 		          // arm origin
const double L_ARM    = 0.889+0.15;                // Updated after experiment
const double D1       = 0.056 + 0.1;
const double D2       = 0.069 + 0.062;
const double THETA1   = atan(D1/L_ARM);//0.174093309;          // arctan(d1/l_arm) computed with l_arm = 0.889m
const double THETA2   = atan(D2/L_ARM);//0.14662888;          // arctan(d2/l_arm) computed with l_arm = 0.889m
const double L1       = sqrt(D1*D1 + L_ARM*L_ARM) ;//0.90258351414;        // sqrt(d_1^2 + arm_len^2)
const double L2       = sqrt(D2*D2 + L_ARM*L_ARM) ;//0.89860002225;        // sqrt(d_2^2 + arm_len^2)
const double TOLERANCE	= 0.00			; 		    // Dont let the arm go closer than TOLERANCE m to the geometric limit
const double P1[2]		= {0.0625+0.1, 0.962}; 	// Half arm width + gearbox width; arm lenght
const double P2[2]		= {-0.0625   , 0.962};	// Symmetric, no gearbox
