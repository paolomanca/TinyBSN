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
        interface Packet as SerialPack;
        interface PacketAcknowledgements;
        interface AMSend;
        interface SplitControl as Radio;
        interface SplitControl as Serial;
        interface AMSend as SerialSend;
        interface Receive;
        interface Timer<TMilli> as MilliTimer;
        interface Timer<TMilli> as Timeout;
        interface Read<uint16_t> as AccSensor;
        interface SplitControl as AccSensorS;
        interface Read<uint16_t> as ECGSensor;
        interface SplitControl as ECGSensorS;
    }

} implementation {


    /*
     * Useful variable for loops
     */
    uint8_t i;

    /*
     * Counter for the number of timeouts (useful for debug purposes)
     */
    uint8_t timeouts = 0;

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
    bsn_msg_t* msg;

    /*
     * Last sent message
     */
    message_t packet;


    task void start();
    task void sendStart();
    task void classify();
    task void sendClass();
    

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
     * This task sends the command START to each PN. Individual messages are sent instead of
     * broadcasting one to get acknowledges. In case of no ack, it can be called again.
     * Only the CN should call this task.
     */
    task void sendStart() {

        // Composing the message
        msg = (bsn_msg_t*)(call Packet.getPayload(&packet,sizeof(bsn_msg_t)));
        msg->msg_type = REQ;
        msg->msg_id = msg_count++;
        msg->value = START;

        dbg("radio_send", "[%s] Trying to send START command to node %d.\n", sim_time_string(), count+1);

        call PacketAcknowledgements.requestAck( &packet );

        if(call AMSend.send(count+1,&packet,sizeof(bsn_msg_t)) == SUCCESS) {

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

	    if ( call Timeout.isRunning() == TRUE ) {
            msg = (bsn_msg_t*)(call Packet.getPayload(&packet,sizeof(bsn_msg_t)));
            msg->msg_type = RES;
            msg->msg_id = msg_count++;
            msg->value = class[0];

            dbg("radio_send", "[%s] Trying to send classification to node 0.\n", sim_time_string());

            call PacketAcknowledgements.requestAck( &packet );

            if(call AMSend.send(0,&packet,sizeof(bsn_msg_t)) == SUCCESS){

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
        } else {
            dbg("role", "[%s] Can't send classification, time is out.", sim_time_string());
        }

    }


    /*
     * This function manages the receiving of classifications from the PNs.
     * Only the CN should call this function.
     */
    void recClass(message_t* pack) {

        if ( call Timeout.isRunning() == TRUE ) {
            class[msg->value]++;
            count++;

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

            dbg_clear("role_coarse", " from node %d (%d of %d).\n", call AMPacket.source(pack), count, N_PNS);

            if ( count == N_PNS ) {
                dbg("role", " Last classification received!\n");

                call Timeout.stop();

                call ECGSensor.read();
            } else {
                dbg_clear("role", "\n");
            }

        }

    }

    
    /*
     * This task sends the application output through the serial port.
     * Only the CN should call this task.
     */
    task void sendAppOut() {

        // Composing the message
        test_serial_msg_t* msg_s = (test_serial_msg_t*)(call SerialPack.getPayload(&packet,sizeof(test_serial_msg_t)));
        //msg->msg_id = msg_count++;
        msg_s->sample_value = class[0];

        dbg("radio_send", "[%s] Trying to send application output on the serial port.\n", sim_time_string());

        if(call SerialSend.send(AM_BROADCAST_ADDR,&packet,sizeof(test_serial_msg_t)) == SUCCESS) {
            dbg("radio_send", "[%s] Packet passed to lower layer successfully!\n", sim_time_string());
        }

    }


    /*
     * BOOT INTERFACE
     */
    event void Boot.booted() {
        dbg("boot","[%s] Application booted.\n", sim_time_string());

        // Starting the radio
        call Radio.start();

        // Starting the sensor
        if ( TOS_NODE_ID == 0 ) {
            call ECGSensorS.start();
            call Serial.start();
        } else {
            call AccSensorS.start();
        }
    }


    /*
     * RADIO
     *
     * This event is received when the radio is started.
     * - CN: trigger the starting procedure of the acquisition
     * - PN: 
     */
    event void Radio.startDone(error_t err){

        if(err == SUCCESS) {

            dbg("radio","[%s] Radio started!\n", sim_time_string());

            if ( TOS_NODE_ID == 0 ) {
                post start();
            }

        } else {
            call Radio.start();
        }

    }

    event void Radio.stopDone(error_t err){
        dbg("radio", "[%s] Radio stopped!\n", sim_time_string());
    }

    /*
     * This event is received when a packet has been sent. Check for acks and eventually trigger new
     * transmissions
     * - CN: deal with START transmission
     * - PN: deal with classification transmission
     */
    event void AMSend.sendDone(message_t* buf,error_t err) {

        if( buf == &packet && err == SUCCESS ) {

            dbg("radio_send", "[%s] Packet sent to %d...", sim_time_string(), call AMPacket.destination(buf));

            if ( call PacketAcknowledgements.wasAcked(buf) ) {
                dbg_clear("radio_ack", "and ack received.\n");

                if ( TOS_NODE_ID == 0 ) {
                    // START command ackwnoledged

                    // Increase the counter to move to the next PN
                    count++;

                    if ( count < N_PNS) {
                        // Send to next PN (if any)
                        post sendStart();
                    } else {
                        // or reset the counter for the next acquisition
                        count = 0;
                    }
                } else {
                    // Classification send ackwnoledged
                    call Timeout.stop();
                }

            } else {
                dbg_clear("radio_ack", "but ack was not received.\n");

                if ( TOS_NODE_ID == 0 ) {
                    // START command NOT ackwnoledged

                    // Resend to same PN
                    post sendStart();
                } else {
                    // Classification send NOT ackwnoledged

                    // Resend classification to CN
                    post sendClass();
                }
            }
        }

    }

    /*
     * This is event is received when a packet is received.
     */
    event message_t* Receive.receive(message_t* buf,void* payload, uint8_t len) {

        msg = (bsn_msg_t*) payload;

        dbg("radio_rec","[%s] Message received.\n", sim_time_string());
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
            // Received classification from a PN

            if ( count == 0 ) {
                call Timeout.startOneShot(CN_TOUT);
            }

            recClass(buf);
        } else {
            // Received START command from the CN

            dbg("role", "[%s] Starting acquisition.\n", sim_time_string());

            if ( call MilliTimer.isRunning() == FALSE ) {
                // Resetting the samples counter
                count = 0;

                dbg("role_fine", "[%s] Starting frequency for acquisition.\n", sim_time_string());
                call MilliTimer.startPeriodic(F_ACQ);

                dbg("role_fine", "[%s] Starting timer for timeout.\n", sim_time_string());
                call Timeout.startOneShot(PN_TOUT);

            } else {
                dbg("role", "[%s] Acquisition already started!\n", sim_time_string());
            }
        }

        return buf;

    }
  
    /*
     * SERIAL PORT
     */
    event void Serial.startDone(error_t err) {
        if ( err == SUCCESS ) {
            dbg("app_out", "Serial port started.\n");
        }
    }
    
    event void Serial.stopDone(error_t err) {}

    event void SerialSend.sendDone(message_t* buf,error_t err) {

        if( err == SUCCESS ) {
            dbg("radio_send", "[%s] Serial packet sent.\n", sim_time_string());
        }
    }


    /*
     * ACCELEROMETER (PNs)
     *
     * This event is received when the read from the accelerometer is complete. It stores the data
     * in the buffer and, if it is full, calls the classification procedure.
     */
    event void AccSensor.readDone(error_t result, uint16_t data) {

        if ( count < BUF_SIZE ) {
            dbg("role_fine", "[%s] Sensed new data from accelerometer. Storing value %d in buffer at position %d\n", sim_time_string(), data, count);
            buffer[count] = data;
            count++;
        } else {
            dbg("role", "[%s] Buffer full! Stopping the acquisition...\n", sim_time_string());

            // Stop the timer to stop getting samples
            call MilliTimer.stop();

            // and classify what we got
            post classify();
        }

    }

    event void AccSensorS.startDone(error_t err) {
        if ( err == SUCCESS ) {
            dbg("role", "[%s] Accelerometer started!\n", sim_time_string());
        } else {
            call AccSensorS.start();
        }
    }

    event void AccSensorS.stopDone(error_t err) {
        dbg("role", "[%s] Accelerometer stopped!\n", sim_time_string());
    }
  

    /*
     * ECG (CN)
     *
     * This event is received when the read from the ECG is complete. It first analyzes the
     * the accelerometers' classifications and, if needed, combines its data with them to determine
     * the application output. Finally, it calls for another acquisition.
     */
    event void ECGSensor.readDone(error_t result, uint16_t data) {

        if ( class[MOVEMENT]+class[CRISIS] >= 3 ) {
            dbg("role_coarse", "[%s] At least 3 nodes detected MOVEMENT or CRISIS, checking ECG...\n", sim_time_string());

            if(data == 1) {
                dbg("role_coarse", "[%s] Heart Rate variation detected ", sim_time_string());

                if ( class[CRISIS] >= 2 ) {
                    dbg_clear("role_coarse", "and at least two nodes detected CRISIS.\n");
                    dbg("app_out", "[%s] Application output: ALARM!\n", sim_time_string());
                    class[0] = CRISIS;
                } else {
                    dbg_clear("role_coarse", "but less than two nodes detected CRISIS.\n");
                    dbg("app_out", "[%s] Application output: MOVEMENT.\n", sim_time_string());
                    class[0] = MOVEMENT;
                }

            } else {
                dbg("role_coarse", "[%s] No Heart Rate variation detected.\n", sim_time_string());
                dbg("app_out", "[%s] Application output: NO_MOVEMENT.\n", sim_time_string());
                class[0] = NO_MOVEMENT;
            }

        } else {
            dbg("role_coarse", "[%s] Less than 3 nodes detected MOVEMENT or CRISIS, no need to check ECG.\n", sim_time_string());
            dbg("app_out", "[%s] Application output: NO_MOVEMENT.\n", sim_time_string());
            class[0] = NO_MOVEMENT;
        }    

        post sendAppOut();
        post start();
    }

    event void ECGSensorS.startDone(error_t err) {
        if ( err == SUCCESS ) {
            dbg("role", "[%s] ECG started!\n", sim_time_string());
        } else {
            call ECGSensorS.start();
        }
    }

    event void ECGSensorS.stopDone(error_t err) {
        dbg("role", "[%s] ECG stopped!\n", sim_time_string());
    }

    
    /*
     * FREQUENCY TIMER (PN)
     *
     * This timer is fired periodically to signal when it's time to get new samples from the accelerometer
     */
    event void MilliTimer.fired() {
        dbg("role_fine", "[%s] Timer fired: reading from the accelerometer...\n", sim_time_string());
        call AccSensor.read();
    }                 


    /*
     * TIMEOUT TIMER (CN & PN)
     *
     * - CN: too much time's passed waiting classification results from PNs, starting from the moment
     * the first one was received
     * - PN: too much time spent for this acquisition since its start
     */
    event void Timeout.fired() {
        timeouts++;
        if ( TOS_NODE_ID == 0 ) {
            dbg("role_coarse", "[%s] Timeout (%d): at least one node failed to deliver in time. Calling another acquisition.\n", sim_time_string(), timeouts);

            post start();
        } else {
            dbg("role", "[%s] Timeout (%d): missed the acquisition, will wait for the next one.\n", sim_time_string(), timeouts);
        }
    }

}
