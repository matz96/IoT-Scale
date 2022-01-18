/*
 * piregler.c
 *
 *  Created on: Nov 30, 2021
 *      Author: Daniel Moser
 */
#include "piregler.h"
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

static float integrate(S_piregler *me);
static float limit(float val, float highlim, float lowlim);

/*
 * Initialisation of the controller
 */
void piregler_init(S_piregler *me, float idlevalue, float val, float kp,
		float mem, float ki, float low, float high, float ts) {
	me->idlevalue = idlevalue;
	me->val = val;
	me->kp = kp;
	me->mem = mem;
	me->ki = ki;
	me->low = low;
	me->high = high;
	me->ts = ts;
}

/*
 * Integration
 */
static float integrate(S_piregler *me) {
	static float integral = 0;
	integral  += me->ki*(((me->mem + me->error)/2)*me->ts);
	integral = limit(integral,me->high,me->low);
	me->mem = me->error;
	return integral;

}

/*
 * Limit for Regler
 */
static float limit(float val, float highlim, float lowlim) {
	if (val < lowlim)
		return lowlim;
	if (val > highlim)
		return highlim;
	return (val);
}

/*
 * calc regler
 */
float ctl_pi(S_piregler *me) {
	me->error = me->idlevalue - me->val;
	return (me->error*me->kp+integrate(me));
}

