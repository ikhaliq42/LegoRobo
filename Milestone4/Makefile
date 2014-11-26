default:
	g++ SURF.cpp Robot.cpp Pose.cpp Globals.cpp Handlers.cpp -o SURF `pkg-config opencv --cflags --libs` -lphidget21 -lboost_system -lboost_thread
blink:
	g++ blinky.cpp -o blinky `pkg-config opencv --cflags --libs` -lphidget21
hlo:
	g++ hello.cpp -o hello `pkg-config opencv --cflags --libs` -lphidget21
