/**
 *  Configuration file for wiring of FakeSensorP module to other common 
 *  components to simulate the behavior of a real sensor
 *
 *  @author Luca Pietro Borsani
 */
 
generic configuration ACCSensorC() {

	provides interface Read<uint16_t>;
	provides interface SplitControl;

} implementation {

	components MainC;
	components new ACCSensorP();
	components new TimerMilliC() as ReadTimer;
	components new TimerMilliC() as InitTimer;
	
	//Connects the provided interface
	Read = ACCSensorP.Read;
	SplitControl = ACCSensorP.SplitControl;

	//Timer interface	
	ACCSensorP.TimerRead -> ReadTimer;
	ACCSensorP.TimerInit -> InitTimer;

}
