// Workspace limits
const double X_LIM[2]	= {.0, 2.750}	; 		// [m] 2 m 6 mm
const double Y_LIM[2]	= {.0, 100.0}	; 		// [m] y has no upper bound
const double X_OFFSET	= 0.165			; 		// Translation from global origin to
const double Y_OFFSET	= 0.1025 + 0.05		; 		// arm origin
const double TOLERANCE	= 0.00			; 		// Dont let the arm go closer than TOLERANCE m to the geometric limit
const double P1[2]		= {0.0625+0.1, 0.962}; 	// Half arm width + gearbox width; arm lenght
const double P2[2]		= {-0.0625   , 0.962};	// Symmetric, no gearbox
