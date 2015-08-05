/*
 *  Configuration file for wiring of TinyBSNC module to other common 
 *  components needed for proper functioning
 *
 *  @author Paolo Manca
 */

#include "TinyBSN.h"

configuration TinyBSNAppC {}

implementation {
    components MainC, TinyBSNC as App;
    components new AMSenderC(AM_BSN_MSG);
    components new AMReceiverC(AM_BSN_MSG);
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
    App.SerialSend -> SerialAM.AMSend[AM_BSN_MSG];

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
