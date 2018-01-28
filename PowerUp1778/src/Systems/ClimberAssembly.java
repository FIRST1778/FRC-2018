package Systems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import NetworkComm.InputOutputComm;
import Utility.HardwareIDs;
import edu.wpi.first.wpilibj.Joystick;

public class ClimberAssembly {
	
	private static boolean initialized = false;
	
	public static void initialize() {
		
		if (initialized)
			return;
		
		InputOutputComm.initialize();
		
		climberMotor = new TalonSRX(HardwareIDs.CLIMBER_TALON_ID);
				
		gamepad = new Joystick(HardwareIDs.GAMEPAD_ID);
		
		initialized = true;
	}
			
	private static final double CLIMBER_MOTOR_DEAD_ZONE = 0.1;

	// apply speed conversion factor from joystick to motor
	// UP joystick = NEG motor throttle = correct!
	private static final double CLIMB_MOTOR_FACTOR = 1.0;
	
	private static TalonSRX climberMotor;
		
	private static Joystick gamepad;
	private static double currentClimbValue = 0.0;
				
	public static void teleopInit() {
		currentClimbValue = 0.0;
	}
	
	public static void teleopPeriodic() {
		

		double newClimbValue = gamepad.getRawAxis(HardwareIDs.CLIMBER_MOTOR_AXIS);
		
		// convert joystick value into motor speed value
		if ((Math.abs(newClimbValue) >= CLIMBER_MOTOR_DEAD_ZONE) && (newClimbValue < 0.0))
			newClimbValue *= CLIMB_MOTOR_FACTOR; 
		else 
			newClimbValue = 0.0;
		
		// if current climber motor is set to this value already, just return
		if (newClimbValue == currentClimbValue)
			return;
		
		// set motor and persisted climb value
		currentClimbValue = newClimbValue;
		climberMotor.set(ControlMode.PercentOutput,newClimbValue);
		
		String climbValueStr = String.format("%.2f", newClimbValue);
		InputOutputComm.putString(InputOutputComm.LogTable.kMainLog,"Climber/speed", climbValueStr);

	}
}
