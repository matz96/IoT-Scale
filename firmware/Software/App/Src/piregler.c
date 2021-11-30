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

void piregler_init(S_piregler *me, float val, float kp, float mem, float tn, float low, float high, float bias, float ts){
	me->val =  val;
	me->kp =  kp;
	me->mem =  mem;
	me->tn =  tn;
	me->low =  low;
	me->high =  high;
	me->bias =  bias;
	me->ts =  ts;
}

/*
 * Integration
 */
static float integrate(S_piregler *me){
	me->mem = me->mem + me->ts*me->val;
	return me->mem;
}

/*
 * Anti-Windup for integrator
 */
static float limit(float val, float highlim, float lowlim){
	if(val < lowlim) return lowlim;
	if(val > highlim)return highlim;
	return(val);
}

float ctl_pi(S_piregler *me){
	me->val = me->val* me->kp;
	return(limit(integrate(me)+me->bias+me->val,me->high,me->low));
}
