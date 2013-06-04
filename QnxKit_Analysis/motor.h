#ifndef motor_h
#define motor_h

#define MOTOR_ON	1
#define MOTOR_OFF	0

// Motor parameters
//#define GR					(50.0)		// gear ratio
//#define ENC_RAD_PER_CNT    	(0.012566) 	// rad/count
#define GR					(100.0)		// gear ratio
#define ENC_RAD_PER_CNT    	(0.0062831853) 	// rad/count

//#define AMP_GAIN			(.00625)   	// V/mA
//#define AMP_GAIN			(6.25)   	// V/A
#define AMP_GAIN			(9.09)   	// V/A

//#define MAX_CURRENT_MA	(1600)		// mA, peak current of motor
//#define MAX_CURRENT_MA		(1.6)		// A, peak current of motor
#define MAX_CURRENT_MA		(1.1)		// A, peak current of motor

#define TORQUE_CONSTANT		(1.11)		// Nm/A
#define INERTIA_MOTOR		(0.0037)	// kg m^2
#define INERTIA				(0.00906)	// (MOTOR + HAND)


// new motor modeling
/*#define KM_POS				(1.933)		// Nm/A for positive torques
#define DYN_FRIC_POS		(0.04092)	// dynamic friction
#define STAT_FRIC_POS		(0.1590)	// static friction

#define KM_NEG				(2.084)		// Nm/A for negative torques
#define DYN_FRIC_NEG		(0.05094)	// dynamic friction
#define STAT_FRIC_NEG		(0.1062)	// static friction
*/
// new values 
// considered the motor's inertia with Phillip's data
#define KM_POS				(1.959)		// Nm/A for positive torques
#define DYN_FRIC_POS		(0.0361)	// dynamic friction
#define STAT_FRIC_POS		(0.1812)	// static friction

#define KM_NEG				(1.909)		// Nm/A for negative torques
#define DYN_FRIC_NEG		(0.03928)	// dynamic friction
#define STAT_FRIC_NEG		(0.1927)	// static friction


// New values from a simple experiment
// These values are compatible with the direction used in the code
/*#define KM_POS				(0.6253)		// Nm/A for positive torques
#define DYN_FRIC_POS		(0.03765)	// dynamic friction
#define STAT_FRIC_POS		(0.01711)	// static friction

#define KM_NEG				(1.0945)		// Nm/A for negative torques
#define DYN_FRIC_NEG		(0.03845)	// dynamic friction
#define STAT_FRIC_NEG		(0.01429)	// static friction
*/

// original
/*#define KM_POS				(0.460)		// Nm/A for positive torques
#define DYN_FRIC_POS		(0.00550)	// dynamic friction
#define STAT_FRIC_POS		(0.0484)	// static friction

#define KM_NEG				(0.437)		// Nm/A for negative torques
#define DYN_FRIC_NEG		(0.00610)	// dynamic friction
#define STAT_FRIC_NEG		(0.0466)	// static friction
*/

// determined only for Philip's 150mA data
/*#define KM_POS			    (0.45708)		// Nm/A for positive torques
#define DYN_FRIC_POS		(0.008066)	// dynamic friction
#define STAT_FRIC_POS		(0.038047)	// static friction

#define KM_NEG				(0.44814)		// Nm/A for negative torques
#define DYN_FRIC_NEG		(0.008127)	// dynamic friction
#define STAT_FRIC_NEG		(0.038229)	// static friction
*/

/*#define KM_POS			(0.2427989)//(0.39) //(0.2427989)		// Nm/A for positive torques
#define DYN_FRIC_POS		(0.0100528)//(0.011) //(0.0100528)	// dynamic friction
#define STAT_FRIC_POS		(0.014614349)//(0.015) //(0.014614349)	// static friction

#define KM_NEG				(0.2427989)//(0.39) //(0.2427989)		// Nm/A for negative torques
#define DYN_FRIC_NEG		(0.0100528)//(0.011)	// dynamic friction
#define STAT_FRIC_NEG		(0.014614349)//(0.015)	// static friction
*/

double calcI(double vel, double accel);
double sign(double vel);

#endif // motor_h

