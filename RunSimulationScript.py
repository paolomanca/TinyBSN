print "********************************************";
print "*                                          *";
print "*             TOSSIM Script                *";
print "*                                          *";
print "********************************************";

import sys;
import time;

from TOSSIM import *;

t = Tossim([]);

sf = SerialForwarder(9001);
throttle = Throttle(t, 10);
sf_process=True;
sf_throttle=True;

topofile="topology.txt";
modelfile="meyer-heavy.txt";


print "Initializing mac....";
mac = t.mac();
print "Initializing radio channels....";
radio=t.radio();
print "    using topology file:",topofile;
print "    using noise file:",modelfile;
print "Initializing simulator....";
t.init();


#simulation_outfile = "simulation.txt";
#print "Saving sensors simulation output to:", simulation_outfile;
#simulation_out = open(simulation_outfile, "w");

#out = open(simulation_outfile, "w");
out = sys.stdout;


#print "Activate debug message on channel verbose"
#t.addChannel("verbose", out);
print "Activate debug message on channel info"
t.addChannel("info", out);
print "Activate debug message on channel main"
t.addChannel("main", out);
print "Activate debug message on channel warn"
t.addChannel("warn", out);
print "Activate debug message on channel app_out"
t.addChannel("app_out",out);

print "\n"

for i in range(0,5):
	print "Creating node %d ..." % (i);
	node1 =t.getNode(i);
	time1 = 0*t.ticksPerSecond();
	node1.bootAtTime(time1);
	print ">>>Will boot at time",  time1/t.ticksPerSecond(), "[sec]";

print "\n"

print "Creating radio channels..."
f = open(topofile, "r");
lines = f.readlines()
for line in lines:
  s = line.split()
  if (len(s) > 0):
    print ">>>Setting radio channel from node ", s[0], " to node ", s[1], " with gain ", s[2], " dBm"
    radio.add(int(s[0]), int(s[1]), float(s[2]))

print "\n"

#Creazione del modello di canale
print "Initializing Closest Pattern Matching (CPM)...";
noise = open(modelfile, "r")
lines = noise.readlines()
compl = 0;
mid_compl = 0;

print "Reading noise model data file:", modelfile;
print "Loading:",
for line in lines:
    str = line.strip()
    if (str != "") and ( compl < 10000 ):
        val = int(str)
        mid_compl = mid_compl + 1;
        if ( mid_compl > 5000 ):
            compl = compl + mid_compl;
            mid_compl = 0;
            sys.stdout.write ("#")
            sys.stdout.flush()
        for i in range(0,5):
            t.getNode(i).addNoiseTraceReading(val)
print "Done!";

for i in range(0,5):
    print ">>>Creating noise model for node:",i;
    t.getNode(i).createNoiseModel()

print "\n"

print "Start simulation with TOSSIM! \n\n";

if ( sf_process == True ):
	sf.process();
if ( sf_throttle == True ):
	throttle.initialize();

while t.time() <= 130*t.ticksPerSecond():
    t.runNextEvent()
    if ( sf_throttle == True ):
	    throttle.checkThrottle();
    if ( sf_process == True ):
	    sf.process();

print "\n\nSimulation finished!";

throttle.printStatistics()

