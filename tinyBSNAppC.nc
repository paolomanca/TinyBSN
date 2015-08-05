/*
 *  Configuration file for wiring of tinyBSNC module to other common 
 *  components needed for proper functioning
 *
 *  Code managing the comunications from PNs to the CN's taken from the
 *  module sendAck by Luca Pietro Borsani.
 *
 *  @author Paolo Manca
 */

#include "tinyBSN.h"

configuration tinyBSNAppC {}

implementation {
    components MainC, tinyBSNC as App;
    components new AMSenderC(AM_MY_MSG);
    components new AMReceiverC(AM_MY_MSG);
    components ActiveMessageC;
    components new TimerMilliC() as TMilli;
    components new TimerMilliC() as Timeout;
    components new ACCSensorC() as Accelerometer;
    components new ECGSensorC() as ECG;

    components SerialActiveMessageC as SerialAM;
    

    //Boot interface
    App.Boot -> MainC.Boot;

    //Send and Receive interfaces
    App.Receive -> AMReceiverC;
    App.AMSend -> AMSenderC;

    //Radio Control
    App.Radio -> ActiveMessageC;

    // Serial Port
    App.Serial -> SerialAM;
    App.SerialPack -> SerialAM;
    App.SerialSend -> SerialAM.AMSend[AM_TEST_SERIAL_MSG];

    //Interfaces to access package fields
    App.AMPacket -> AMSenderC;
    App.Packet -> AMSenderC;
    App.PacketAcknowledgements->ActiveMessageC;

    //Timer interface
    App.MilliTimer -> TMilli;
    App.Timeout -> Timeout;

    //Sensors
    App.AccSensor -> Accelerometer;
    App.AccSensorS -> Accelerometer;
    App.ECGSensor -> ECG;
    App.ECGSensorS -> ECG;

}
