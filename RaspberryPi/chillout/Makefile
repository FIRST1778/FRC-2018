all:	chillOutObjTracker calibrateTracker usbcvstream usbstream

clean:
	rm -f chillOutObjTracker calibrateTracker usbcvstream usbstream

INCLUDE_DIRS = -I /home/pi/ntcore/src/main/native/include -I /home/pi/wpiutil/src/main/native/include -I/home/pi/CmakeWpilib/libraries/cscore/cscore/src/main/native/include
LIB_DIRS = -L /home/pi/ntcore/build/libs/ntcore/shared -L /home/pi/wpiutil/build/libs/wpiutil/shared -L/home/pi/CmakeWpilib/libraries/cscore

chillOutObjTracker: chillOutObjTracker.cpp
	g++ -o chillOutObjTracker chillOutObjTracker.cpp $(INCLUDE_DIRS) `pkg-config opencv --cflags --libs` -std=gnu++11  $(LIB_DIRS) -lwpiutil -lntcore -lv4l2

calibrateTracker: calibrateTracker.cpp
	g++ -o calibrateTracker calibrateTracker.cpp $(INCLUDE_DIRS) `pkg-config opencv --cflags --libs` -std=gnu++11  $(LIB_DIRS) -lwpiutil -lntcore -lv4l2

usbcvstream: usbcvstream.cpp
	g++ -o usbcvstream usbcvstream.cpp $(INCLUDE_DIRS) `pkg-config opencv --cflags --libs` -std=gnu++11  $(LIB_DIRS) -lwpiutil -lntcore -lcscore -lv4l2

usbstream: usbstream.cpp
	g++ -o usbstream usbstream.cpp $(INCLUDE_DIRS) `pkg-config opencv --cflags --libs` -std=gnu++11  $(LIB_DIRS) -lwpiutil -lntcore -lcscore -lv4l2
