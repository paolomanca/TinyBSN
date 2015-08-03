#include "DATA/ECG_DATA/ECG_DATA_1.h"
 
generic module ECGSensorP() {

	provides interface Read<uint16_t>;
	provides interface SplitControl;

	uses interface Timer<TMilli> as TimerRead;
	uses interface Timer<TMilli> as TimerInit;

} implementation {

	uint16_t temp_index;
	uint16_t read_value=0;

	command error_t SplitControl.start(){
		temp_index = 0;
		call TimerInit.startOneShot( 2 );
		return SUCCESS;
	}

	command error_t SplitControl.stop(){
		return SUCCESS;
	}

	//***************** Read interface ********************//
	command error_t Read.read(){

		read_value = DATA[temp_index];
		temp_index++;
		if(temp_index==ECG_DATA_SIZE)
			temp_index = 0;

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
