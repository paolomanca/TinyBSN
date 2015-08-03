/**
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
    components new TimerMilliC();
    components new FakeSensorC() as Accelerometer;
    components new FakeSensorC() as ECG;

    //Boot interface
    App.Boot -> MainC.Boot;

    //Send and Receive interfaces
    App.Receive -> AMReceiverC;
    App.AMSend -> AMSenderC;

    //Radio Control
    App.SplitControl -> ActiveMessageC;

    //Interfaces to access package fields
    App.AMPacket -> AMSenderC;
    App.Packet -> AMSenderC;
    App.PacketAcknowledgements->ActiveMessageC;

    //Timer interface
    App.MilliTimer -> TimerMilliC;

    //Sensors
    App.AccSensor -> Accelerometer;
    App.ECGSensor -> ECG;

}
