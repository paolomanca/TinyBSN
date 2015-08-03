#include "DATA/ACC_DATA/ACC_DATA_1.h"
 
generic module ACCSensorP() {

	provides interface Read<uint16_t>;
	provides interface SplitControl;

	uses interface Timer<TMilli> as TimerRead;
	uses interface Timer<TMilli> as TimerInit;

} implementation {

	uint16_t temp_index[4];
	uint16_t read_value=0;

	command error_t SplitControl.start(){
		uint8_t idx = 0;
		for(idx=0;idx<4;idx++){
			temp_index[idx] = 0;
		}
		call TimerInit.startOneShot( 2 );
		return SUCCESS;
	}

	command error_t SplitControl.stop(){
		return SUCCESS;
	}

	//***************** Read interface ********************//
	command error_t Read.read(){
		switch(TOS_NODE_ID){
			case 1:
				read_value = DATA1[temp_index[0]];
				temp_index[0]++;
				if(temp_index[0]==ACC_DATA_SIZE)
					temp_index[0] = 0;
				break;
			case 2:
				read_value = DATA2[temp_index[1]];
				temp_index[1]++;
				if(temp_index[1]==ACC_DATA_SIZE)
					temp_index[1] = 0;
				break;
			case 3:
				read_value = DATA3[temp_index[2]];
				temp_index[2]++;
				if(temp_index[2]==ACC_DATA_SIZE)
					temp_index[2] = 0;
				break;
			case 4:
				read_value = DATA4[temp_index[3]];
				temp_index[3]++;
				if(temp_index[3]==ACC_DATA_SIZE)
					temp_index[3] = 0;
				break;
			default:
				read_value = 0;
		}
		call TimerRead.startOneShot( 2 );
		return SUCCESS;
	}

	//***************** Timer interfaces ********************//
	event void TimerInit.fired() {
		signal SplitControl.startDone( SUCCESS );
	}

	event void TimerRead.fired() {
		signal Read.readDone( SUCCESS, read_value );
	}
}
