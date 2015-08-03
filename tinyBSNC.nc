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
    interface SplitControl as AccSensorS;
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
     * Message variable
     */
    my_msg_t* msg;

    /*
     * Last sent message
     */
    message_t packet;


    task void start();
    task void classify();
    task void sendStart();
    task void sendClass();
    
    
    /*
     * This task send the command START to each PN. It sends individual messages instead of
     * broadcasting to get acknowledges. In case of no ack, it can be called again.
     * Only the CN should call this task.
     */
    task void sendStart() {

        // Composing the message
        msg=(my_msg_t*)(call Packet.getPayload(&packet,sizeof(my_msg_t)));
        msg->msg_type = REQ;
        msg->msg_id = msg_count++;
        msg->value = START;

        dbg("radio_send", "[%s] Trying to send START command to node %d.\n", sim_time_string(), count+1);

        call PacketAcknowledgements.requestAck( &packet );

        if(call AMSend.send(count+1,&packet,sizeof(my_msg_t)) == SUCCESS) {

            dbg("radio_send", "[%s] Packet passed to lower layer successfully!\n", sim_time_string());
            dbg("radio_pack",">>>Pack\n \t Payload length %hhu \n",
                call Packet.payloadLength(&packet) );
            dbg_clear("radio_pack","\t Source: %hhu \n ", call AMPacket.source( &packet ) );
            dbg_clear("radio_pack","\t Destination: %hhu \n ",
                call AMPacket.destination( &packet ) );
            dbg_clear("radio_pack","\t AM Type: %hhu \n ", call AMPacket.type( &packet ) );
            dbg_clear("radio_pack","\t\t Payload \n" );
            dbg_clear("radio_pack", "\t\t msg_type: %hhu \n ", msg->msg_type);
            dbg_clear("radio_pack", "\t\t msg_id: %hhu \n", msg->msg_id);
            dbg_clear("radio_pack", "\t\t value: %hhu \n", msg->value);
            dbg_clear("radio_pack", "\n");

        }

    }


    /*
     * This task starts the acquisition process.
     * Only the CN should call this task.
     */
     task void start() {
        dbg_clear("role_coarse", "\n");
        dbg("role_coarse", "[%s] New acquisition started!\n", sim_time_string());
        count = 0;

        for ( i=0; i<4; i++ ) {
            class[i] = 0;
        }

        post sendStart();

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

        dbg("role", "[%s] Rescaling samples...\n", sim_time_string());

        for(i=0; i<BUF_SIZE; i++) {
            dbg("role_fine", "Scaling [%d]: %d -> %f\n", i, buffer[i], (float)buffer[i]*10/65535);
            sum += (float)buffer[i]*10/65535; // 2^16-1
        }

        dbg("role_fine", "[%s] Rescaling finished.\n", sim_time_string());

        avg = sum/BUF_SIZE;

        dbg("role", "[%s] Sample average: %f (sum: %f)\n", sim_time_string(), avg, sum);

        // Classification according to the thresholds
        dbg("role", "[%s] Classified as ", sim_time_string());

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

        msg=(my_msg_t*)(call Packet.getPayload(&packet,sizeof(my_msg_t)));
        msg->msg_type = RES;
        msg->msg_id = msg_count++;
        msg->value = class[0];

        dbg("radio_send", "[%s] Trying to send classification to node 0.\n", sim_time_string());

        call PacketAcknowledgements.requestAck( &packet );

        if(call AMSend.send(0,&packet,sizeof(my_msg_t)) == SUCCESS){

            dbg("radio_send", "[%s] Packet passed to lower layer successfully!\n", sim_time_string());
            dbg("radio_pack",">>>Pack\n \t Payload length %hhu \n",
                call Packet.payloadLength(&packet) );
            dbg_clear("radio_pack","\t Source: %hhu \n ", call AMPacket.source( &packet ) );
            dbg_clear("radio_pack","\t Destination: %hhu \n ",
                call AMPacket.destination(&packet) );
            dbg_clear("radio_pack","\t AM Type: %hhu \n ", call AMPacket.type( &packet ) );
            dbg_clear("radio_pack","\t\t Payload \n" );
            dbg_clear("radio_pack", "\t\t msg_type: %hhu \n ", msg->msg_type);
            dbg_clear("radio_pack", "\t\t msg_id: %hhu \n", msg->msg_id);
            dbg_clear("radio_pack", "\t\t value: %hhu \n", msg->value);
            dbg_clear("radio_pack", "\n");

        }

    }


    /*
     * This function manages the receiving of classifications from the PNs.
     * Only the CN should call this function.
     */
    void recClass(message_t* pack) {
        class[msg->value]++;
        count++;

        if ( call MilliTimer.isRunning == FALSE ) {
            call MilliTimer.startOneShot(500);
        }

        dbg("role_coarse", "[%s] Received classification ", sim_time_string());

        switch(msg->value){
            case NO_MOVEMENT:
                dbg_clear("role_coarse", "NO_MOVEMENT");
                break;
            case MOVEMENT:
                dbg_clear("role_coarse", "MOVEMENT");
                break;
            case CRISIS:
                dbg_clear("role_coarse", "CRISIS");
                break;
            default: break;
        }

        dbg_clear("role_coarse", " from node %d (%d of %d).", call AMPacket.source(pack), count, N_PNS);

        if ( count == N_PNS ) {
            dbg_clear("role", " Last classification received!\n");

            call MilliTimer.stop();

            if ( buffer[MOVEMENT]+buffer[CRISIS] >= 3 ) {
                dbg("role_coarse","[%s] At least 3 nodes detected MOVEMENT or CRISIS, getting Heart Rate variation from ECG...\n", sim_time_string());
                call ECGSensor.read();
            } else {
                dbg("role_coarse", "[%s] Less than 3 nodes detected MOVEMENT or CRISIS, calling for another acquisition...\n", sim_time_string());
                post start();
            }
        } else {
            dbg_clear("role", "\n");
        }

    }     

    //***************** Boot interface ********************//
    event void Boot.booted() {
        dbg("boot","[%s] Application booted.\n", sim_time_string());
        call SplitControl.start();
    }

    //***************** SplitControl interface ********************//
    event void SplitControl.startDone(error_t err){

        if(err == SUCCESS) {

            dbg("radio","[%s] Radio on!\n", sim_time_string());

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
        if ( TOS_NODE_ID == 0 ) {
            dbg("role_coarse", "[%s] Timeout: at least one node failed to deliver in time. Calling another acquisition.\n", sim_time_string());
            call start();
        } else {
            dbg("role_fine", "[%s] Timer fired! Time to sense!\n", sim_time_string());
            call AccSensor.read();
        }
    }                 


    //********************* AMSend interface ****************//
    event void AMSend.sendDone(message_t* buf,error_t err) {

        if( buf == &packet && err == SUCCESS ) {

            dbg("radio_send", "[%s] Packet sent to %d...", call AMPacket.destination(buf), sim_time_string());

            if ( call PacketAcknowledgements.wasAcked(buf) ) {
                dbg_clear("radio_ack", "and ack received");

                if ( TOS_NODE_ID == 0 ) {
                    // Ack coming from the START command,
                    count++;

                    // Send to next node (if any)
                    if ( count < N_PNS) {
                        post sendStart();
                    } else {
                        count = 0;
                    }
                }

            } else {
                dbg_clear("radio_ack", "but ack was not received");

                if ( TOS_NODE_ID == 0 ) {
                    // Ack missing from the START command
                    post sendStart();
                } else {
                    // Ack missing from the classification transmission
                    post sendClass();
                }
            }
        }

    }

    //***************************** Receive interface *****************//
    event message_t* Receive.receive(message_t* buf,void* payload, uint8_t len) {

        msg=(my_msg_t*)payload;

        dbg("radio_rec","[%s] Message received.", sim_time_string());
        dbg("radio_pack",">>>Pack \n \t Payload length %hhu \n", call Packet.payloadLength(buf) );
        dbg_clear("radio_pack","\t Source: %hhu \n", call AMPacket.source(buf) );
        dbg_clear("radio_pack","\t Destination: %hhu \n", call AMPacket.destination(buf) );
        dbg_clear("radio_pack","\t AM Type: %hhu \n", call AMPacket.type(buf) );
        dbg_clear("radio_pack","\t\t Payload \n" );
        dbg_clear("radio_pack", "\t\t msg_type: %hhu \n", msg->msg_type);
        dbg_clear("radio_pack", "\t\t msg_id: %hhu \n", msg->msg_id);
        dbg_clear("radio_pack", "\t\t value: %hhu \n", msg->value);
        dbg_clear("radio_pack","\n");

        if ( TOS_NODE_ID == 0 ) {
            recClass(buf);
        } else {

            if ( call MilliTimer.isRunning() == FALSE ) {
                // Resetting the counter
                count = 0;

                dbg("role", "[%s] Starting accelerometer\n", sim_time_string());
                call AccSensorS.start();

                dbg("role_fine", "[%s] Starting timer for acquisition.\n", sim_time_string());
                call MilliTimer.startPeriodic(50); // 20Hz

            } else {
                dbg("role", "[%s] Acquisition already started!\n", sim_time_string());
            }
        }

        return buf;

    }
  
    //************************* Accelerometer Read interface **********************//
    event void AccSensor.readDone(error_t result, uint16_t data) {

        if ( count < BUF_SIZE ) {
            dbg("role_fine", "[%s] Sensed new data from accelerometer. Storing value %d in buffer at position %d\n", sim_time_string(), data, count);
            buffer[count] = data;
            count++;
        } else {
            dbg("role", "[%s] Buffer full!\n", sim_time_string());

            // Stop the timer to stop getting samples
            call MilliTimer.stop();

        dbg("role", "[%s] Stopping accelerometer\n", sim_time_string());
        call AccSensorS.stop();

            // and classify what we got
            post classify();
        }

    }

    event void AccSensorS.startDone(error_t err) {
        dbg("role", "[%s] Accelerometer started!\n", sim_time_string());
    }

    event void AccSensorS.stopDone(error_t err) {
        dbg("role", "[%s] Accelerometer stopped!\n", sim_time_string());
    }
  

  //************************* ECG Read interface **********************//
  event void ECGSensor.readDone(error_t result, uint16_t data) {    
    if(data == 1) {
        dbg("role_coarse", "[%s] Heart Rate variation detected ", sim_time_string());

        if ( class[CRISIS] >= 2 ) {
            dbg_clear("role_coarse", "and at least two nodes detected CRISIS: it's a CRISIS, sending an ALARM!\n");
        } else {
            dbg_clear("role_coarse", "but less than two nodes detected CRISIS: just MOVEMENT.\n");
        }

    } else {
        dbg("role_coarse", "[%s] No Heart Rate variation detected.\n", sim_time_string());
    }

    post start();
  }

}
