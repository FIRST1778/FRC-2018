package Systems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

/* deprecated 2018
import com.ctre.phoenix.MotorControl.SmartMotorController.FeedbackDevice;
import com.ctre.phoenix.MotorControl.SmartMotorController.FeedbackDeviceStatus;
import com.ctre.phoenix.MotorControl.SmartMotorController.TalonControlMode;
*/

import NetworkComm.InputOutputComm;
import Utility.HardwareIDs;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Spark;

public class CubeManagement {
	
	private static boolean initialized = false;
	private static final int TIMEOUT_MS = 0;  // set to zero if skipping confirmation
	private static final int PIDLOOP_IDX = 0;  // set to zero if primary loop
	private static final int PROFILE_SLOT = 0;
	
	private static boolean feeding = false;
		
	private static final double COLLECTOR_IN_LEVEL = -0.75;
	private static final double COLLECTOR_OUT_LEVEL = 0.75;

	private static final double LIFT_UP_LEVEL = -0.75;
	private static final double LIFT_DOWN_LEVEL = 0.75;
	
	private static final double DEAD_ZONE_THRESHOLD = 0.05;
		    
	public static void initialize() {
		
		if (initialized)
			return;
		
		InputOutputComm.initialize();
		
		// reset trigger init time
		initTriggerTime = RobotController.getFPGATime();

        // create pneumatics objects
		compress = new Compressor(HardwareIDs.PCM_ID);
		flipperSolenoid = new DoubleSolenoid(HardwareIDs.PCM_ID, HardwareIDs.FLIPPER_UP_SOLENOID, HardwareIDs.FLIPPER_DOWN_SOLENOID);
            
		
		// create motors
		leftCollectorMotor = new TalonSRX(HardwareIDs.LEFT_COLLECTOR_TALON_ID);
		rightCollectorMotor = new TalonSRX(HardwareIDs.RIGHT_COLLECTOR_TALON_ID);

		leftCollectorMotor.setNeutralMode(NeutralMode.Brake);
		rightCollectorMotor.setNeutralMode(NeutralMode.Brake);
		
		leftLowerLiftMotor = new TalonSRX(HardwareIDs.LEFT_LOWER_LIFT_TALON_ID);
		rightLowerLiftMotor = new TalonSRX(HardwareIDs.LEFT_LOWER_LIFT_TALON_ID);
		leftUpperLiftMotor = new TalonSRX(HardwareIDs.RIGHT_UPPER_LIFT_TALON_ID);
		rightUpperLiftMotor = new TalonSRX(HardwareIDs.RIGHT_UPPER_LIFT_TALON_ID);

		leftLowerLiftMotor.setNeutralMode(NeutralMode.Brake);
		rightLowerLiftMotor.setNeutralMode(NeutralMode.Brake);
		leftUpperLiftMotor.setNeutralMode(NeutralMode.Brake);
		rightUpperLiftMotor.setNeutralMode(NeutralMode.Brake);
		
		// make sure all motors are off
		resetMotors();
		
		gamepad = new Joystick(HardwareIDs.GAMEPAD_ID);
		
		initialized = true;
	}
						
	// collector intake motors
	private static TalonSRX leftCollectorMotor, rightCollectorMotor; 
	
	// lift motors
	private static TalonSRX leftUpperLiftMotor, rightUpperLiftMotor;
	private static TalonSRX leftLowerLiftMotor, rightLowerLiftMotor;
		
	// pneumatics objects
	private static Compressor compress;
	private static DoubleSolenoid flipperSolenoid;
	
	private static Joystick gamepad;
		
	// wait 0.25 s between button pushes on shooter
    private static final int TRIGGER_CYCLE_WAIT_US = 250000;
    private static double initTriggerTime;
    	
	public static void resetMotors()
	{		
		leftCollectorMotor.set(ControlMode.PercentOutput, 0);
		rightCollectorMotor.set(ControlMode.PercentOutput, 0);	
		
		leftLowerLiftMotor.set(ControlMode.PercentOutput, 0);
		leftUpperLiftMotor.set(ControlMode.PercentOutput, 0);
		rightLowerLiftMotor.set(ControlMode.PercentOutput, 0);
		rightUpperLiftMotor.set(ControlMode.PercentOutput, 0);
		
	}
	
	public static void flipperUp()
	{
		flipperSolenoid.set(DoubleSolenoid.Value.kForward);		
	}
	
	public static void flipperDown()
	{
		flipperSolenoid.set(DoubleSolenoid.Value.kReverse);		
	}
		
	private static void checkCollectorControls() {
						
		// collector control
		double collectorLevel = gamepad.getRawAxis(HardwareIDs.COLLECTOR_IN_AXIS);
		if (Math.abs(collectorLevel) > DEAD_ZONE_THRESHOLD)
			collectorLevel = COLLECTOR_IN_LEVEL;
		else if (gamepad.getRawButton(HardwareIDs.COLLECTOR_OUT_BUTTON))
			collectorLevel = COLLECTOR_OUT_LEVEL;
		else
			collectorLevel = 0.0;
		InputOutputComm.putDouble(InputOutputComm.LogTable.kMainLog,"CubeMgmt/CollectorLevel", collectorLevel);
		leftCollectorMotor.set(ControlMode.PercentOutput, collectorLevel);
		rightCollectorMotor.set(ControlMode.PercentOutput, collectorLevel);
		
	}
	
	private static void checkLiftControls() {
		// cube lift control
		double liftLevel = gamepad.getRawAxis(HardwareIDs.LIFT_DOWN_AXIS);
		if (Math.abs(liftLevel) > DEAD_ZONE_THRESHOLD)
			liftLevel = LIFT_DOWN_LEVEL;
		else if (gamepad.getRawButton(HardwareIDs.LIFT_UP_BUTTON))
			liftLevel = LIFT_UP_LEVEL;
		else
			liftLevel = 0.0;
		InputOutputComm.putDouble(InputOutputComm.LogTable.kMainLog,"CubeMgmt/LiftLevel", liftLevel);
		leftLowerLiftMotor.set(ControlMode.PercentOutput, liftLevel);
		rightLowerLiftMotor.set(ControlMode.PercentOutput, liftLevel);
		leftUpperLiftMotor.set(ControlMode.PercentOutput, liftLevel);
		rightUpperLiftMotor.set(ControlMode.PercentOutput, liftLevel);
	}
	
	private static void checkFlipperControls() {
		if (gamepad.getRawButton(HardwareIDs.FLIPPER_UP_BUTTON))
		{
			flipperUp();
		}
		else if (gamepad.getRawButton(HardwareIDs.FLIPPER_DOWN_BUTTON))
		{
			flipperDown();	
		}
	}
	
	public static void autoInit() {
				
		resetMotors();
		flipperUp();
		
        initTriggerTime = RobotController.getFPGATime();
	}

	public static void teleopInit() {
				
		resetMotors();
		flipperUp();
				
        initTriggerTime = RobotController.getFPGATime();
        
	}
	
	public static void teleopPeriodic() {
		
		checkCollectorControls();
		checkLiftControls();
		checkFlipperControls();
	}
}
