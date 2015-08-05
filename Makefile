COMPONENT=TinyBSNAppC
BUILD_EXTRA_DEPS += TinyBSNListener.class
CLEAN_EXTRA = *.class TinyBSNListenerMsg.java

CFLAGS += -I$(TOSDIR)/lib/T2Hack

TinyBSNListener.class: $(wildcard *.java) TinyBSNListenerMsg.java
	javac -target 1.4 -source 1.4 *.java

TinyBSNListenerMsg.java:
	mig java -target=null $(CFLAGS) -java-classname=TinyBSNListenerMsg TinyBSN.h bsn_msg -o $@


include $(MAKERULES)
