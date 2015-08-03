/*
 *  Source file for implementation of module tinyBSN, which implements a Body Sensor Network (BSN)
 *  composed of four wireless accelerometers (two for the wrists, and two for the ankles) and one
 *  wireless ElectroCardioGram (ECG) sensor, mounted on the chest.
 *  The BSN is organized in a star topology, with the ECG sensor acting as the central node (CN).
 *  The CN is also responsible for controlling the peripheral nodes (PN), by triggering the
 *  monitoring process.
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
        interface Read<uint16_t> as ECGSensor;
    }

} implementation {


    /*
     * Useful variable for loops
     */
    uint8_t i;

    /*
     * Counter variable
     * - CN: count responses from PNs
     * - PN: count the number of samples collected
     */
    uint8_t count;

    /*
     * Counter for  the number of messagges sent
     */
    uint8_t msg_count = 0;

    /*
     * - CN: store in position 0 the application result and in the other positions count the
     * occurence of the different classifications received from the PNs
     * - PN: store in position 0 the resulting classification
     */
    uint8_t class[4];

    /*
     * Buffer to store accelerometers' samples
     */
    uint16_t buffer[BUF_SIZE];

    /*
     * Array of last sent messages
     * - CN: for each PN store the last packet sent to it
     * - PN: in position 0 store the last packet sent to the CN
     */
    message_t packets[N_PNS];

    task void start();
    task void classify();
    task void sendStart();
    task void sendClass();
    task void evalClass();
    
    
    /*
     * This task send the command START to each PN. It sends individual messages instead of
     * broadcasting to get acknowledges. In case of no ack, it can be called again.
     * Only the CN should call this task.
     */
    task void sendStart() {

        dbg("radio_send", "Broadcasting START command at time %s \n", sim_time_string());

        for ( i = 0; i<N_PNS; i++ ) {

            // Composing the message
            my_msg_t* mess=(my_msg_t*)(call Packet.getPayload(&packets[i],sizeof(my_msg_t)));
            mess->msg_type = REQ;
            mess->msg_id = msg_count++;
            mess->value = START;

            call PacketAcknowledgements.requestAck( &packets[i] );

            if(call AMSend.send(i+1,&packets[i],sizeof(my_msg_t)) == SUCCESS) {

                dbg("radio_send", "Packet passed to lower layer successfully!\n");
                dbg("radio_pack",">>>Pack\n \t Payload length %hhu \n",
                    call Packet.payloadLength(&packets[i]) );
                dbg_clear("radio_pack","\t Source: %hhu \n ", call AMPacket.source( &packets[i] ) );
                dbg_clear("radio_pack","\t Destination: %hhu \n ",
                    call AMPacket.destination( &packets[i] ) );
                dbg_clear("radio_pack","\t AM Type: %hhu \n ", call AMPacket.type( &packets[i] ) );
                dbg_clear("radio_pack","\t\t Payload \n" );
                dbg_clear("radio_pack", "\t\t msg_type: %hhu \n ", mess->msg_type);
                dbg_clear("radio_pack", "\t\t msg_id: %hhu \n", mess->msg_id);
                dbg_clear("radio_pack", "\t\t value: %hhu \n", mess->value);
                dbg_clear("radio_pack", "\n");

            }
        }

    }


    /*
     * This task starts the acquisition process.
     * Only the CN should call this task.
     */
     task void start() {

        count = 0;

        for ( i=0; i<4; i++ ) {
            class[i] = 0;
        }

        sendStart();

     }


    /*
     * This task rescales the samples, calculates the average and classifies the result. At the end
     * calls sendClass to send the classification to the CN.
     * Only PNs should call this task.
     */
    task void classify() {

        /** Sum of rescaled samples **/ 
        float sum = 0;

        /** Average of rescaled samples **/
        float avg;

        dbg("role", "Rescaling samples...\n");

        for(i=0; i<BUF_SIZE; i++) {
            dbg("role_fine", "Scaling [%d]: %d -> %f\n", i, buffer[i], (float)buffer[i]*10/65535);
            sum += (float)buffer[i]*10/65535; // 2^16-1
        }

        dbg("role_fine", "Rescaling finished.\n");

        avg = sum/BUF_SIZE;

        dbg("role", "Sample average: %f (sum: %f)\n", avg, sum);

        // Classification according to the thresholds
        dbg("role", "Classified as ");

        if ( avg < M_THR ) {
            class[0] = NO_MOVEMENT;
            dbg_clear("role", "NO_MOVEMENT (avg < M_THR)\n");
        } else if ( avg > C_THR ) {
            class[0] = CRISIS;
            dbg_clear("role", "CRISIS (avg > C_THR)\n");
        } else {
            class[0] = MOVEMENT;
            dbg_clear("role", "MOVEMENT (M_THR <= avg <= C_THR)\n");
        }

        // Send the classication to the CN
        post sendClass();
    }


    /*
     * This task sends the PN's classification to the CN.
     * Only PNs should call this task.
     */
    task void sendClass() {

        my_msg_t* mess=(my_msg_t*)(call Packet.getPayload(&packets[0],sizeof(my_msg_t)));
        mess->msg_type = RES;
        mess->msg_id = msg_count++;
        mess->value = class[0];

        dbg("radio_send", "Try to send classification to node 0 at time %s \n", sim_time_string());

        call PacketAcknowledgements.requestAck( &packets[0] );

        if(call AMSend.send(0,&packets[0],sizeof(my_msg_t)) == SUCCESS){

            dbg("radio_send", "Packet passed to lower layer successfully!\n");
            dbg("radio_pack",">>>Pack\n \t Payload length %hhu \n",
                call Packet.payloadLength(&packets[0]) );
            dbg_clear("radio_pack","\t Source: %hhu \n ", call AMPacket.source( &packets[0] ) );
            dbg_clear("radio_pack","\t Destination: %hhu \n ",
                call AMPacket.destination(&packets[0]) );
            dbg_clear("radio_pack","\t AM Type: %hhu \n ", call AMPacket.type( &packets[0] ) );
            dbg_clear("radio_pack","\t\t Payload \n" );
            dbg_clear("radio_pack", "\t\t msg_type: %hhu \n ", mess->msg_type);
            dbg_clear("radio_pack", "\t\t msg_id: %hhu \n", mess->msg_id);
            dbg_clear("radio_pack", "\t\t value: %hhu \n", mess->value);
            dbg_clear("radio_pack", "\n");

        }

    }


    /*
     * This task manages the receiving of classifications from the PNs.
     * Only the CN should call this task.
     */
    task void recClass() {
        class[mess->value]++;
        count++;

        dbg("role", "Received classification ");

        switch(mess->value){
            case NO_MOVEMENT:
                dbg_clear("role", "NO_MOVEMENT");
                break;
            case MOVEMENT:
                dbg_clear("role", "MOVEMENT");
                break;
            case CRISIS:
                dbg_clear("role", "CRISIS");
                break;
            default: break;
        }

        dbg_clear("role", " from node %d (%d of %d)\n", call AMPacket.source(buf), count, N_PNS);

        if ( count == N_PNS ) {
            dbg("role", "Last classification received\n");

            if ( buffer[MOVEMENT]+buffer[CRISIS] >= 3 ) {
                dbg("role","At least 3 nodes detected MOVEMENT or CRISIS, getting Heart Rate variation from ECG...\n");
                call ECGSensor.read();
            } else {
                dbg("role", "Less than 3 nodes detected MOVEMENT or CRISIS, calling for another acquisition...");
                post start();
            }
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

            if ( TOS_NODE_ID == 0 ) {
                post start();
            }

        } else {
            call SplitControl.start();
        }

    }

    event void SplitControl.stopDone(error_t err){}


    //***************** MilliTimer interface ********************//
    event void MilliTimer.fired() {
        dbg("role_fine", "Timer fired! Time to sense!\n");

        call AccSensor.read();
    }                 


    //********************* AMSend interface ****************//
    event void AMSend.sendDone(message_t* buf,error_t err) {

        if( buf == &packets[call AMPacket.destination(buf)] && err == SUCCESS ) {

            if ( call AMPacket.destination( buf ) != AM_BROADCAST_ADDR ) {
                dbg("radio_send", "Packet sent to %d...", call AMPacket.destination(buf));

                if ( call PacketAcknowledgements.wasAcked( buf ) ) {
                    dbg_clear("radio_ack", "and ack received");
                } else {
                    dbg_clear("radio_ack", "but ack was not received");

                        if ( TOS_NODE_ID == 0 ) {
                            post sendStart();
                        } else {
                            post sendClass();
                        }
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

        dbg("radio_rec","Message received at time %s \n", sim_time_string());
        dbg("radio_pack",">>>Pack \n \t Payload length %hhu \n", call Packet.payloadLength(buf) );
        dbg_clear("radio_pack","\t Source: %hhu \n", call AMPacket.source(buf) );
        dbg_clear("radio_pack","\t Destination: %hhu \n", call AMPacket.destination(buf) );
        dbg_clear("radio_pack","\t AM Type: %hhu \n", call AMPacket.type(buf) );
        dbg_clear("radio_pack","\t\t Payload \n" );
        dbg_clear("radio_pack", "\t\t msg_type: %hhu \n", mess->msg_type);
        dbg_clear("radio_pack", "\t\t msg_id: %hhu \n", mess->msg_id);
        dbg_clear("radio_pack", "\t\t value: %hhu \n", mess->value);
        dbg_clear("radio_pack","\n");

        if ( TOS_NODE_ID == 0 ) {
            post recClass();
        } else {

            if ( !MilliTimer.isRunnning() ) {
                // Resetting the counter
                count = 0;

                dbg("role", "Starting timer for acquisition.\n");
                call MilliTimer.startPeriodic(50); // 20Hz

            } else {
                dbg("role", "Acquisition already started");
            }
        }

        return buf;

    }
  
    //************************* Accelerometer Read interface **********************//
    event void AccSensor.readDone(error_t result, uint16_t data) {

        if ( count < BUF_SIZE ) {
            dbg("role_fine", "Sensed new data from accelerometer. Storing value %d in buffer at position %d\n", data, count);
            buffer[count] = data;
            count++;
        } else {
            dbg("role", "Buffer full!\n");

            // Stop the timer to stop getting samples
            call MilliTimer.stop();

            // and classify what we got
            post classify();
        }

    }
  

  //************************* ECG Read interface **********************//
  event void ECGSensor.readDone(error_t result, uint16_t data) {
    if(data == 1) {
        dbg("role", "Heart Rate variation detected ");

        if ( class[CRISIS] >= 2 ) {
            dbg_clear("role", "and at least two nodes detected CRISIS: it's a CRISIS, sending an ALARM!\n");
        } else {
            dbg_clear("role", "but less than two nodes detected CRISIS: just MOVEMENT.\n");
        }

    } else {
        dbg("role", "No Heart Rate variation detected.\n");
    }

    post start();
  }

}
