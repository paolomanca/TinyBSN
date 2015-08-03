﻿/**
 *  @author Paolo Manca
 */

#ifndef SENDACK_H
#define SENDACK_H

#define BUF_SIZE 200

#define N_PNS 4

/** Classification thresholds **/
#define M_THR 0.5
#define C_THR 2

/** Message types **/
#define REQ 1
#define RES 2

/** Message values **/
#define START 0
#define NO_MOVEMENT 1
#define MOVEMENT 2
#define CRISIS 3

typedef nx_struct my_msg {
		nx_uint8_t msg_type;
		nx_uint16_t msg_id;
		nx_uint8_t value;
} my_msg_t;



enum{
	AM_MY_MSG = 6,
};

#endif
