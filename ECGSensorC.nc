/**
 *  Configuration file for wiring of FakeSensorP module to other common 
 *  components to simulate the behavior of a real sensor
 *
 *  @author Luca Pietro Borsani
 */

generic configuration ECGSensorC() {

	provides interface Read<uint16_t>;
	provides interface SplitControl;

} implementation {

	components MainC;
	components new ECGSensorP();
	components new TimerMilliC() as ReadTimer;
	components new TimerMilliC() as InitTimer;
	
	//Connects the provided interface
	Read = ECGSensorP.Read;
	SplitControl = ECGSensorP.SplitControl;

	//Timer interface	
	ECGSensorP.TimerRead -> ReadTimer;
	ECGSensorP.TimerInit -> InitTimer;

}
