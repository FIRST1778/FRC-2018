package FreezyDrive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;


public class FreezyDriveTrain {
	
	// TalonSRX IDs
	private static final int LEFT_FRONT_TALON_ID = 3;
	private static final int LEFT_REAR_TALON_ID = 7;
	private static final int RIGHT_FRONT_TALON_ID = 8;
	private static final int RIGHT_REAR_TALON_ID = 4;

	// Initalizing TalonSRXs
	private static TalonSRX motorFrontL,motorFrontR;
	private static TalonSRX motorRearL,motorRearR;

	private static DriveControl driveControl;
	private static boolean initialized = false;
	
	public static void initialize() {
		if (initialized)
			return;
		
		motorFrontL = new TalonSRX(LEFT_FRONT_TALON_ID);
		motorFrontR = new TalonSRX(RIGHT_FRONT_TALON_ID);
		motorRearL = new TalonSRX(LEFT_REAR_TALON_ID);
		motorRearR = new TalonSRX(RIGHT_REAR_TALON_ID);
				
		driveControl = new DriveControl();
		
		Controller.initialize();
		
		initialized = true;
	}
			
		
	// call to change the power given to the motor
	public static void ChangeSpeed(double powerL,double powerR){
		motorFrontL.set(ControlMode.PercentOutput,powerL);
		motorRearL.set(ControlMode.PercentOutput, powerL);
		motorFrontR.set(ControlMode.PercentOutput, powerR);
		motorRearR.set(ControlMode.PercentOutput, powerR);
	}
	
	public static void teleopInit()
	{
		
	}
	
	public static void teleopPeriodic()
	{
    	// drive command for all controllers
   	 	driveControl.calculateDrive(Controller.Driver_Throttle(), Controller.Driver_Steering(),
   	 		Controller.Driver_isQuickTurn());
		
	}
}