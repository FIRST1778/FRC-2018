package Systems;

import NetworkComm.InputOutputComm;
import Utility.HardwareIDs;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;

public class CubeManagement {
	
	private static boolean initialized = false;
    
	public static void initialize() {
		
		if (initialized)
			return;
		
		InputOutputComm.initialize();
		
		// reset trigger init time
		initTriggerTime = RobotController.getFPGATime();		
		
		gamepad = new Joystick(HardwareIDs.DRIVER_CONTROL_ID);
				
		initialized = true;
	}
	
	// wait 0.25 s between button pushes
    private static final int TRIGGER_CYCLE_WAIT_US = 250000;
    private static double initTriggerTime;
				
	private static Joystick gamepad;
	
	
	public static void autoInit() {
		
	}
	
	public static void teleopInit() {
		
	}
	
	public static void teleopPeriodic() {

		// fire controls - using a timer to debounce
		double currentTime = RobotController.getFPGATime();
				
		// reset trigger init time
		initTriggerTime = RobotController.getFPGATime();		

	}
}
