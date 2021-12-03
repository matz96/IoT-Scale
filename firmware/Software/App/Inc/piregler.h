/*
 * piregler.h
 *
 *  Created on: Nov 30, 2021
 *      Author: Daniel Moser
 */

#ifndef PIREGLER_H_
#define PIREGLER_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

typedef struct piregler{
	float idlevalue;
	float val;
	float kp;
	float mem;
	float tn;
	float low;
	float high;
	float bias;
	float ts;
}S_piregler;

void piregler_init(S_piregler *me,float idlevalue, float val, float kp, float mem, float tn, float low, float high, float bias, float ts);
float ctl_pi(S_piregler *me);
#endif /* PIREGLER_H_ */


