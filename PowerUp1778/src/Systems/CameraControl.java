package Systems;

import NetworkComm.InputOutputComm;
import Utility.HardwareIDs;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.RobotController;

public class CameraControl {
	
	private static boolean initialized = false;
    
	public static void initialize() {
		
		if (initialized)
			return;
		
		InputOutputComm.initialize();
		
		// reset trigger init time
		initTriggerTime = RobotController.getFPGATime();		

		cameraLedRelay = new Relay(HardwareIDs.CAMERA_LED_RELAY_CHANNEL,Relay.Direction.kForward);
		cameraLedRelay.set(Relay.Value.kOff);
				
		gamepad = new Joystick(HardwareIDs.DRIVER_CONTROL_ID);
		
		ledState = false;
		
		initialized = true;
	}
				
	private static Joystick gamepad;
			
	// Relay for extra LEDs
	private static Relay cameraLedRelay;
	private static boolean ledState = false;
	
	// wait 0.25 s between button pushes
    private static final int TRIGGER_CYCLE_WAIT_US = 250000;
    private static double initTriggerTime;
		
	public static void setCameraLed(boolean state) {
		
		if (state == true) {
			cameraLedRelay.set(Relay.Value.kOn);
			ledState = true;
		}
		else {
			cameraLedRelay.set(Relay.Value.kOff);
			ledState = false;
		}
		
	}
	
	public static void autoInit() {
		
		setCameraLed(true);
	}
	
	public static void teleopInit() {
		
		setCameraLed(false);
	}
	
	public static void teleopPeriodic() {

		// fire controls - using a timer to debounce
		double currentTime = RobotController.getFPGATime();

		// if not enough time has passed, no polling allowed!
		if ((currentTime - initTriggerTime) < TRIGGER_CYCLE_WAIT_US)
			return;
		
		// button to strobe camera LED rings
		if ((gamepad.getRawButton(HardwareIDs.CAMERA_LED_STROBE_BUTTON) == true) && (!ledState))
		{
			setCameraLed(true);
		}
		else if ((gamepad.getRawButton(HardwareIDs.CAMERA_LED_STROBE_BUTTON) == false) && (ledState))
		{
			setCameraLed(false);
		}
		
		// reset trigger init time
		initTriggerTime = RobotController.getFPGATime();		

	}
}
