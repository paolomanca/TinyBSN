/*
 * Header file for the TinyBSN
 * @author Paolo Manca
 */

#ifndef SENDACK_H
#define SENDACK_H

/** Number of Peripheral Nodes **/
#define N_PNS 4

/** Number of samples for each acquisition **/
#define BUF_SIZE 200

/** Classification thresholds **/
#define M_THR 0.5
#define C_THR 2.0

/** Message types **/
#define REQ 1
#define RES 2

/** Message values **/
#define START 0
#define NO_MOVEMENT 1
#define MOVEMENT 2
#define CRISIS 3

typedef nx_struct bsn_msg {
		nx_uint8_t msg_type;
		nx_uint16_t msg_id;
		nx_uint8_t value;
} bsn_msg_t;

enum {
  AM_BSN_MSG = 0x89,
};

/*
 * Timers' durations (in ms)
 */

/** Frequency of PN's acquisitions **/
#define F_ACQ 50 // 20Hz = 50ms

/** PN max time for acquisition **/
#define PN_TOUT 15000

/** CN wait from the first received classification before abort **/
#define CN_TOUT 15000


#endif
