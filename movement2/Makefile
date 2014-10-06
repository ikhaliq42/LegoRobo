default:
	g++ DriveControl.cpp -o DriveControl -lphidget21
	g++ InterfaceControl.cpp -o InterfaceControl -lphidget21 
	g++ Robot.cpp -o Robot -lphidget21	
	g++ MoveAndAvoid.cpp -o MoveAndAvoid `pkg-config opencv --cflags --libs`
