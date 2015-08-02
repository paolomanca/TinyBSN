/**
 *  Source file for implementation of module tinyBSN, which implements
 *  a Body Sensor Network (BSN) composed of four wireless accelerometers
 *  (two for the wrists, and two for the ankles) and one wireless
 *  ElectroCardioGram (ECG) sensor, mounted on the chest.
 *  The BSN is organized in a star topology, with the ECG sensor acting
 *  as the central node (CN). The CN is also responsible for controlling
 *  the peripheral nodes (PN), by triggering the monitoring process.
 *
 *  Code managing the comunications from PNs to the CN's taken from the
 *  module sendAck by Luca Pietro Borsani.
 *
 *  @author Paolo Manca
 * 
 */

#include "tinyBSN.h"
#include "Timer.h"

module tinyBSNC {

  uses {
	interface Boot;
    	interface AMPacket;
	interface Packet;
	interface PacketAcknowledgements;
    	interface AMSend;
    	interface SplitControl;
    	interface Receive;
    	interface Timer<TMilli> as MilliTimer;
	interface Read<uint16_t> as AccSensor;
  }

} implementation {

  uint16_t buffer[BUF_SIZE];
  uint8_t buf_count = 0;

  uint8_t msg_count = 0;

  uint8_t rec_id;
  message_t packet;

  task void start();
  
  
  //***************** Task start acquisition *****************//
  task void start() {

	my_msg_t* mess=(my_msg_t*)(call Packet.getPayload(&packet,sizeof(my_msg_t)));
	mess->msg_type = REQ;
	mess->msg_id = msg_count++;
	mess->value = START;
	    
	dbg("radio_send", "Trying to broadcast START command at time %s \n", sim_time_string());
    
	call PacketAcknowledgements.requestAck( &packet );

	if(call AMSend.send(AM_BROADCAST_ADDR,&packet,sizeof(my_msg_t)) == SUCCESS){
		
	  dbg("radio_send", "Broadcast packet passed to lower layer successfully!\n");
	  dbg("radio_pack",">>>Pack\n \t Payload length %hhu \n", call Packet.payloadLength( &packet ) );
	  dbg_clear("radio_pack","\t Source: %hhu \n ", call AMPacket.source( &packet ) );
	  dbg_clear("radio_pack","\t Destination: %hhu \n ", call AMPacket.destination( &packet ) );
	  dbg_clear("radio_pack","\t AM Type: %hhu \n ", call AMPacket.type( &packet ) );
	  dbg_clear("radio_pack","\t\t Payload \n" );
	  dbg_clear("radio_pack", "\t\t msg_type: %hhu \n ", mess->msg_type);
	  dbg_clear("radio_pack", "\t\t msg_id: %hhu \n", mess->msg_id);
	  dbg_clear("radio_pack", "\t\t value: %hhu \n", mess->value);
	  dbg_clear("radio_pack", "\n");
      
      }
  }

  //***************** Task sense from accelerometer *****************//
  task void classify() {
	dbg("role", "Started classification");
  }

  //***************** Task send request ********************//
  task void sendReq() {

	my_msg_t* mess=(my_msg_t*)(call Packet.getPayload(&packet,sizeof(my_msg_t)));
	mess->msg_type = REQ;
	mess->msg_id = msg_count++;
	    
	dbg("radio_send", "Try to send a request to node 0 at time %s \n", sim_time_string());
    
	call PacketAcknowledgements.requestAck( &packet );

	if(call AMSend.send(0,&packet,sizeof(my_msg_t)) == SUCCESS){
		
	  dbg("radio_send", "Packet passed to lower layer successfully!\n");
	  dbg("radio_pack",">>>Pack\n \t Payload length %hhu \n", call Packet.payloadLength( &packet ) );
	  dbg_clear("radio_pack","\t Source: %hhu \n ", call AMPacket.source( &packet ) );
	  dbg_clear("radio_pack","\t Destination: %hhu \n ", call AMPacket.destination( &packet ) );
	  dbg_clear("radio_pack","\t AM Type: %hhu \n ", call AMPacket.type( &packet ) );
	  dbg_clear("radio_pack","\t\t Payload \n" );
	  dbg_clear("radio_pack", "\t\t msg_type: %hhu \n ", mess->msg_type);
	  dbg_clear("radio_pack", "\t\t msg_id: %hhu \n", mess->msg_id);
	  dbg_clear("radio_pack", "\t\t value: %hhu \n", mess->value);
	  dbg_clear("radio_pack", "\n");
      
      }

 }        

  //***************** Boot interface ********************//
  event void Boot.booted() {
	dbg("boot","Application booted.\n");
	call SplitControl.start();
  }

  //***************** SplitControl interface ********************//
  event void SplitControl.startDone(error_t err){
      
    if(err == SUCCESS) {

	dbg("radio","Radio on!\n");
	//if ( TOS_NODE_ID != 0 ) {
	//  dbg("role","I'm node %d: start sending periodical request\n", TOS_NODE_ID);
	//  call MilliTimer.startPeriodic( 800 );
	//}

	if ( TOS_NODE_ID == 0 ) {
		post start();
	}
    }
    else{
	call SplitControl.start();
    }

  }
  
  event void SplitControl.stopDone(error_t err){}

  //***************** MilliTimer interface ********************//
  event void MilliTimer.fired() {
	dbg("role", "Timer fired! Time to sense!\n");
	if(TOS_NODE_ID != 0) {
		call AccSensor.read();
	}
  }
  

  //********************* AMSend interface ****************//
  event void AMSend.sendDone(message_t* buf,error_t err) {

    if(&packet == buf && err == SUCCESS ) {

	if ( call AMPacket.destination( buf ) != AM_BROADCAST_ADDR ) {
		dbg("radio_send", "Packet sent to %d...", call AMPacket.destination( buf ));

		if ( call PacketAcknowledgements.wasAcked( buf ) ) {
		  dbg_clear("radio_ack", "and ack received");
		} else {
		  dbg_clear("radio_ack", "but ack was not received");
		}
		dbg_clear("radio_send", " at time %s \n\n", sim_time_string());
	} else {
		dbg("radio_send", "Packet broadcasted at time %s \n", sim_time_string());
	}

    }

  }

  //***************************** Receive interface *****************//
  event message_t* Receive.receive(message_t* buf,void* payload, uint8_t len) {

	my_msg_t* mess=(my_msg_t*)payload;
	rec_id = mess->msg_id;
	
	dbg("radio_rec","Message received at time %s \n", sim_time_string());
	dbg("radio_pack",">>>Pack \n \t Payload length %hhu \n", call Packet.payloadLength( buf ) );
	dbg_clear("radio_pack","\t Source: %hhu \n", call AMPacket.source( buf ) );
	dbg_clear("radio_pack","\t Destination: %hhu \n", call AMPacket.destination( buf ) );
	dbg_clear("radio_pack","\t AM Type: %hhu \n", call AMPacket.type( buf ) );
	dbg_clear("radio_pack","\t\t Payload \n" );
	dbg_clear("radio_pack", "\t\t msg_type: %hhu \n", mess->msg_type);
	dbg_clear("radio_pack", "\t\t msg_id: %hhu \n", mess->msg_id);
	dbg_clear("radio_pack", "\t\t value: %hhu \n", mess->value);
	dbg_clear("radio_pack","\n");
	
	if ( TOS_NODE_ID != 0 ) {
		if ( mess->msg_type == REQ && mess->value == START ) {
			dbg("role", "Starting the timer. \n");
			call MilliTimer.startPeriodic(500); // 20Hz
		}
	} else {
	}

    return buf;

  }
  
  //************************* Read interface **********************//
  event void AccSensor.readDone(error_t result, uint16_t data) {
	if ( buf_count < BUF_SIZE ) {
		dbg("role", "Sensed new data from accelerometer. Storing it in buffer at position %d\n", buf_count);
		buffer[buf_count] = data;
		buf_count++;
	} else {
		dbg("role", "Buffer full");
		post classify();
		buf_count = 0;
		call MilliTimer.stop();
	}
  }

}

