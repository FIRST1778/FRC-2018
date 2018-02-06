package Utility;

public class HardwareIDs {
	
	// CAN Bus Hardware IDs (note: never use ID=1, it is factory default)
	public static final int PCM_ID = 2;
	
	public static final int LEFT_FRONT_TALON_ID = 3;
	public static final int LEFT_REAR_TALON_ID = 7;
	public static final int RIGHT_FRONT_TALON_ID = 8;
	public static final int RIGHT_REAR_TALON_ID = 4;

	public static final int LOWER_LIFT_TALON_ID = 5;
	public static final int UPPER_LIFT_TALON_ID = 6;
	
	public static final int CLIMBER_TALON_ID = 9;
	
	// Collector and Climber Sparks (PWM)
	public static final int LEFT_COLLECTOR_PWM_ID = 2;
	public static final int RIGHT_COLLECTOR_PWM_ID = 3;
	
	// Solenoids (pneumatics)
	public static final int FLIPPER_UP_SOLENOID = 0;
	public static final int FLIPPER_DOWN_SOLENOID = 1;
	
	public static final int CLAMP_ON_SOLENOID = 2;
	public static final int CLAMP_OFF_SOLENOID = 7;
	
	public static final int BRAKE_ON_SOLENOID = 4;
	public static final int BRAKE_OFF_SOLENOID = 5;

	// input control IDs
	public static final int DRIVER_CONTROL_ID = 0;
	public static final int GAMEPAD_ID = 1;
	
	// copilot (gamepad) controls
	public static final int COLLECTOR_IN_AXIS = 2;
	public static final int COLLECTOR_OUT_BUTTON = 5;
		
	public static final int FLIPPER_DOWN_AXIS = 3;
	public static final int FLIPPER_UP_BUTTON = 6;

	public static final int BRAKE_TOGGLE_BUTTON = 10;

	public static final int LIFT_MOTOR_AXIS = 5;

	public static final int CLIMBER_MOTOR_AXIS = 1;

	// Digital IO (DIO) channels
	public static final int TRIGGER_CHANNEL_ID = 0;
	public static final int ECHO_CHANNEL_ID = 1;	
	
	// Encoder Constants
	
	// original encoder variables
	//private static final int ENCODER_PULSES_PER_REV = 250;  // E4P-250  - on the proto bot front motors
	//private static final double INCHES_PER_REV = (6 * 3.14159);   // 6-in diameter wheel
	
	//public static final int ENCODER_PULSES_PER_REV = 256;  // 63R  - on the competition bot motors
	public static final int ENCODER_PULSES_PER_REV = 256*4;  // 63R  - on the competition bot motors

	//public static final double INCHES_PER_REV = (6 * 3.14159);   // 6-in diameter wheel (theoretical)
	public static final double INCHES_PER_REV = (5.9 * 3.14159);   // 5.9-in diameter wheel (worn)
			
	public static final double INCHES_PER_ENCODER_PULSE = INCHES_PER_REV/ENCODER_PULSES_PER_REV;
	public static final double RPM_TO_UNIT_PER_100MS = ENCODER_PULSES_PER_REV/(60*10);

	
}
