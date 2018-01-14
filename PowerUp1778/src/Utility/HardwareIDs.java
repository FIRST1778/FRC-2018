package Utility;

public class HardwareIDs {
	
	// CAN Bus Hardware IDs (note: never use ID=1, it is factory default)
	public static final int PCM_ID = 2;
	
	public static final int LEFT_FRONT_TALON_ID = 3;
	public static final int LEFT_REAR_TALON_ID = 7;
	public static final int RIGHT_FRONT_TALON_ID = 8;
	public static final int RIGHT_REAR_TALON_ID = 4;

	public static final int LEFT_COLLECTOR_TALON_ID = 10;
	public static final int RIGHT_COLLECTOR_TALON_ID = 9;
	
	public static final int LEFT_LOWER_LIFT_TALON_ID = 5;
	public static final int RIGHT_LOWER_LIFT_TALON_ID = 6;
	public static final int LEFT_UPPER_LIFT_TALON_ID = 11;
	public static final int RIGHT_UPPER_LIFT_TALON_ID = 12;
	
	public static final int CLIMBER_TALON_ID = 13;
	
	// Solenoids (pneumatics)
	public static final int FLIPPER_UP_SOLENOID = 0;
	public static final int FLIPPER_DOWN_SOLENOID = 1;

	// input control IDs
	public static final int DRIVER_CONTROL_ID = 0;
	public static final int GAMEPAD_ID = 1;
	
	// copilot (gamepad) controls
	public static final int COLLECTOR_IN_AXIS = 2;
	public static final int COLLECTOR_OUT_BUTTON = 5;
	
	public static final int FLIPPER_UP_BUTTON = 4;
	public static final int FLIPPER_DOWN_BUTTON = 1;
	
	public static final int LIFT_DOWN_AXIS = 3;
	public static final int LIFT_UP_BUTTON = 6;

	public static final int CLIMBER_MOTOR_AXIS = 1;

	// Digital IO (DIO) channels
	public static final int TRIGGER_CHANNEL_ID = 0;
	public static final int ECHO_CHANNEL_ID = 1;	
	

	
}
