/*
 * file: pid_ma.h
 *
 * Library which contains the moving average filter and the PID controller implementation.
 * Multiple instances of both can be implemented by just creating multiple "ma_t" or "pid_t".
 */

#ifndef PID_MA_H
#define PID_MA_H

#include <stdint.h>

/***********************/
/* MA FILTER typedefs */
/***********************/
/* structure where the MA parameters are stored */
typedef struct
{
	float avg;
	float N;
}ma_t;

/***************************/
/* PID controller typedefs */
/***************************/
/* structure where the PID parameters are stored */
typedef struct
{
	float a, b, c; // a,b,c are the PID formula coefficients initialized in config_PID
	float u, r; // u: PID output // r: PID reference
	float e_n, e_n1, e_n2;
	/*"e_n" is the error in the current iteration
	 * "e_n1" is the error from the previous iteration
	 * "e_n2" is the error of the antepenultimate iteration */

	uint32_t min_output; // output saturation limits
	uint32_t max_output;
}pid_t;


/**********************************/
/* MA FILTER function prototypes */
/**********************************/
/* config_MA
 *
 * @brief: initializes an ma_t structure by setting the samples number
 * @param: MA: structure which contains the average and parameters
 * @param: samples: number of samples of the MA
 */
void config_MA(volatile ma_t * MA, const uint32_t samples);

/* compute_MA
 *
 * @brief: given a new value, computes the new average of an specific MA
 * @param: MA: structure which contains the average and parameters
 * @param: newVal: new value to compute the new average
 */
void compute_MA(volatile ma_t * MA, const float newVal);

/**************************************/
/* PID controller function prototypes */
/**************************************/
/* config_PID
 *
 * @brief: sets the parameters of the controller
 * @param: PID: structure where the PID data are stored
 * @param: Kp: proportional gain
 * @param: Ki: integral gain
 * @param: Kd: derivative gain
 * @param: Ts: time between iterations
 * @param: MaxOut: maximum PID output
 * @param: MinOut: minimum PID output
 */
void config_PID(volatile pid_t * PID, const float Kp, const float Ki, const float Kd, const float Ts, const int32_t MaxOut, const int32_t MinOut);

/* start_PID
 *
 * @brief: resets some critical PID variables
 * @param: PID: structure where the PID data are stored
 */
void start_PID(volatile pid_t * PID);

/* stop_PID
 *
 * @brief: resets some critical PID variables
 * @param: PID: structure where the PID data are stored
 */
void stop_PID(volatile pid_t * PID);

/* compute_PID
 *
 * @brief: given an input, computes the new PID ouput
 * @param: PID: structure where the PID data are stored
 * @param: y: PID sensor value
 */
void compute_PID(volatile pid_t * PID, volatile const float y);

#endif
