/**
 *  @author Luca Pietro Borsani
 */

#ifndef SENDACK_H
#define SENDACK_H

#define BUF_SIZE 200

typedef nx_struct my_msg {
	nx_uint8_t msg_type;
	nx_uint16_t msg_id;
	nx_uint8_t value;
} my_msg_t;

#define REQ 1
#define RESP 2

#define START 0
#define NO_MOVEMENT 1
#define MOVEMENT 2
#define CRISIS 3

enum{
AM_MY_MSG = 6,
};

#endif
