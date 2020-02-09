/*
 * file: pid_ma.c
 *
 * Library which contains the moving average filter and the PID controller implementation.
 * Multiple instances of both can be implemented by just creating multiple "ma_t" or "pid_t".
 */

/* includes */
#include "pid_ma.h"

/***********************************/
/* EMA FILTER function definitions */
/***********************************/
/* config_MA
 *
 * @brief: initializes an ma_t structure by setting the samples number
 * @param: MA: structure which contains the average and parameters
 * @param: samples: number of samples of the MA
 */
void config_MA(volatile ma_t * MA, const uint32_t samples)
{
	MA->N = (float)samples;
}

/* compute_MA
 *
 * @brief: given a new value, computes the new average of an specific MA
 * @param: MA: structure which contains the average and parameters
 * @param: newVal: new value to compute the new average
 */
void compute_MA(volatile ma_t * MA, const float newVal)
{
	MA->avg = MA->avg*(1-(1.0f/MA->N)) + newVal*(1.0f/MA->N);
}

/***************************************/
/* PID controller function definitions */
/***************************************/

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
void config_PID(volatile pid_t * PID, const float Kp, const float Ki, const float Kd, const float Ts, const int32_t MaxOut, const int32_t MinOut)
{
	PID->a = ( Kp + Ki*Ts/2 + Kd/Ts);
	PID->b = (-Kp + Ki*Ts/2 -2*Kd/Ts);
	PID->c = Kd/Ts;
	PID->max_output = MaxOut;
	PID->min_output = MinOut;
}

/* start_PID
 *
 * @brief: resets some critical PID variables
 * @param: PID: structure where the PID data are stored
 */
void start_PID(volatile pid_t * PID)
{
	PID->u = 0;
	PID->r = 0;
	PID->e_n  = 0;
	PID->e_n1 = 0;
	PID->e_n2 = 0;
}

/* stop_PID
 *
 * @brief: resets some critical PID variables
 * @param: PID: structure where the PID data are stored
 */
void stop_PID(volatile pid_t * PID)
{
	PID->u = 0;
	PID->r = 0;
}

/* compute_PID
 *
 * @brief: given an input, computes the new PID ouput
 * @param: PID: structure where the PID data are stored
 * @param: y: PID sensor value
 */
void compute_PID(volatile pid_t * PID, volatile const float y)
{
	/* errors are updated */
	PID->e_n2 = PID->e_n1;
	PID->e_n1 = PID->e_n;
	PID->e_n = PID->r - y;

	PID->u = PID->u + PID->a*PID->e_n + PID->b*PID->e_n1 + PID->c*PID->e_n2;

	if(PID->u > PID->max_output)
		PID->u = PID->max_output;
	else if(PID->u < PID->min_output)
		PID->u = PID->min_output;
}

