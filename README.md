# TinyBSN
A Body Sensor Network (BSN) for detecting epileptic seizures composed of four wireless accelerometers (two for the wrists, and two for the ankles) and one wireless ElectroCardioGram (ECG) sensor, mounted on the chest. The BSN is organized in a star topology, with the ECG sensor acting as the central node (CN). The CN is also responsible for controlling the peripheral nodes (PN), by triggering the monitoring process.

Code managing the comunications from PNs to the CN's taken from the module sendAck by Luca Pietro Borsani.

## Folders structure
```
TinyBSN
|
├── .gitignore
├── DATA
|   ├── ACC_DATA: accelerometer's data
|   |   ├── ACC_DATA_1.h: data for the "Teethbrushing" scenario
|   |   ├── ACC_DATA_2.h: data for the "Walking" scenario
|   |   └── ACC_DATA_3.h: data for the "Crisis during sleep" scenario
|   └── ECG_DATA: ECG's data
|       ├── ECG_DATA_1.h: data for the "Teethbrushing" scenario
|       ├── ECG_DATA_2.h: data for the "Walking" scenario
|       └── ECG_DATA_3.h: data for the "Crisis during sleep" scenario
|
├── ACCSensorC.np: configuration for the accelerometer module
├── ACCSensorP.np: implementation of the accelerometer module
├── ECGSensorC.np: configuration for the ECG module
├── ECGSensorP.np: implementation of the ECG module
├── Makefile
├── README.md
├── RunSimulationScript.py: pythone script that runs the simulation
├── TinyBSN.h: header file for the BSN module
├── TinyBSNAppC.np: configuration for the BSN module
├── TinyBSNC.np: implementation of the BSN module
├── TinyBSNListener.java: client listening for the serial port
├── meyer-heavy.txt: noise model
└── topology.txt: network topology
```
## How to run (in TOSSIM + TOSSIM Live)
Inside the main folder of the project, compile for TOSSIM:
```
make micaz sim-sf
```
Run the serial forwarder:
```
java net.tinyos.sf.SerialForwarder -comm sf@localhost:9001&
```
Run the client
```
java TinyBSNListener -comm sf@localhost:9002
```

Open another terminal in the main folder of the project, and start the simulation:
```
python RunSimulationScript.py
```
