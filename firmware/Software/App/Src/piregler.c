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

void piregler_init(S_piregler *me, float idlevalue, float val, float kp, float mem, float tn, float low, float high, float bias, float ts){
	me->idlevalue = idlevalue;
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
	float integral  = me->error + ((1/me->tn)*(((me->mem + me->error)/2)*me->ts));
	me->mem = me->error;
	return integral;
}

/*
 * Limit for Regler
 */
static float limit(float val, float highlim, float lowlim){
	if(val < lowlim) return lowlim;
	if(val > highlim)return highlim;
	return(val);
}

float ctl_pi(S_piregler *me){
	me->error = me->val - me->idlevalue;
	return(me->kp*integrate(me));
}

