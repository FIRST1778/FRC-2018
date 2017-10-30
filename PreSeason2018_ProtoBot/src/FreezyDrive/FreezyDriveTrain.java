package FreezyDrive;

import com.ctre.phoenix.MotorControl.CAN.TalonSRX;


public class FreezyDriveTrain {
	
    // singleton class elements (ensures only one instance of this class)
	private static final FreezyDriveTrain instance = new FreezyDriveTrain();
    
	private FreezyDriveTrain() {
		motorFrontL = new TalonSRX(LEFT_FRONT_TALON_ID);
		motorFrontR = new TalonSRX(RIGHT_FRONT_TALON_ID);
		motorRearL = new TalonSRX(LEFT_REAR_TALON_ID);
		motorRearR = new TalonSRX(RIGHT_REAR_TALON_ID);
		
		driveControl = new DriveControl();
		
		Controller.initialize();
	}
		
	public static FreezyDriveTrain GetInstance() {
		return instance;
	}
	
	// TalonSRX IDs
	private static final int LEFT_FRONT_TALON_ID = 3;
	private static final int LEFT_REAR_TALON_ID = 7;
	private static final int RIGHT_FRONT_TALON_ID = 8;
	private static final int RIGHT_REAR_TALON_ID = 4;

	// Initalizing TalonSRXs
	private TalonSRX motorFrontL,motorFrontR;
	private TalonSRX motorRearL,motorRearR;

	private DriveControl driveControl;
		
	// call to change the power given to the motor
	public void ChangeSpeed(double powerL,double powerR){
		motorFrontL.set(powerL);
		motorRearL.set(powerL);
		motorFrontR.set(powerR);
		motorRearR.set(powerR);
	}
	
	public void teleopInit()
	{
		
	}
	
	public void teleopPeriodic()
	{
    	// drive command for all controllers
   	 	driveControl.calculateDrive(Controller.Driver_Throttle(), Controller.Driver_Steering(),
   	 		Controller.Driver_isQuickTurn());
		
	}
}