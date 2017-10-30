package Systems;

import com.ctre.phoenix.MotorControl.CAN.TalonSRX;

import NetworkComm.InputOutputComm;
import Utility.HardwareIDs;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Spark;

public class ClimberAssembly {
	
    // singleton class elements (ensures only one instance of this class)
	private static final ClimberAssembly instance = new ClimberAssembly();
    
	private ClimberAssembly() {
		
		ioComm = InputOutputComm.GetInstance();
		
		climberMotor = new TalonSRX(HardwareIDs.CLIMBER_TALON_ID);
		
		//************ DEBUG only - PROTOBOT ***************
		//climberMotor = new Spark(HardwareIDs.PROTOBOT_PWM_ID);
				
		gamepad = new Joystick(HardwareIDs.GAMEPAD_ID);
	}
		
	public static ClimberAssembly GetInstance() {
		return instance;
	}
	
	private static final double CLIMBER_MOTOR_DEAD_ZONE = 0.1;

	// apply speed conversion factor from joystick to motor
	// UP joystick = NEG motor throttle = correct!
	private static final double CLIMB_MOTOR_FACTOR = 1.0;
	
	// COMPETITION BOT climber
	private InputOutputComm ioComm;
	private TalonSRX climberMotor;
	
	//******* DEBUG climber only (PROTOBOT)******
	//private static Spark climberMotor;
	//*******************************************
	
	private Joystick gamepad;
	private double currentClimbValue = 0.0;
				
	public void teleopInit() {
		currentClimbValue = 0.0;
	}
	
	public void teleopPeriodic() {

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
		climberMotor.set(newClimbValue);
		
		String climbValueStr = String.format("%.2f", newClimbValue);
		ioComm.putString(InputOutputComm.LogTable.kMainLog,"Climber/speed", climbValueStr);
	}
}
