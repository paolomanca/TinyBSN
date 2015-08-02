# TinyBSN
A Body Sensor Network (BSN) for detecting epileptic seizures composed of four wireless accelerometers (two for the wrists, and two for the ankles) and one wireless ElectroCardioGram (ECG) sensor, mounted on the chest. The BSN is organized in a star topology, with the ECG sensor acting as the central node (CN). The CN is also responsible for controlling the peripheral nodes (PN), by triggering the monitoring process.

Code managing the comunications from PNs to the CN's taken from the module sendAck by Luca Pietro Borsani.
