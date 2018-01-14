package Systems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import NetworkComm.InputOutputComm;
import Utility.HardwareIDs;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;

public class CubeManagement {
	
	private static boolean initialized = false;
	private static final int TIMEOUT_MS = 0;  // set to zero if skipping confirmation
	private static final int PIDLOOP_IDX = 0;  // set to zero if primary loop
	private static final int PROFILE_SLOT = 0;

	// auto lift levels
	public static final int BASE_LEVEL = 0;
	public static final int SWITCH_LEVEL = 1;
	public static final int SCALE_LEVEL = 2;
	
	private static final int liftLevelPulses[][] = {{0,0}, {100, 100}, {200, 200}};  // number of upper and lower encoder pulses for each level  {upper, lower}
	private static final int liftMotorVelocity = 900;  // native units per 100ms
	private static final int liftMotorAccel = 450;  // native units per 100ms ^2
	
	// motor polarity
	public static final boolean RIGHT_COLLECTOR_REVERSE_MOTOR = false; 
	public static final boolean LEFT_COLLECTOR_REVERSE_MOTOR = false; 
	
	public static final boolean RIGHT_UPPER_REVERSE_MOTOR = false; 
	public static final boolean RIGHT_LOWER_REVERSE_MOTOR = false; 
	public static final boolean LEFT_UPPER_REVERSE_MOTOR = false;
	public static final boolean LEFT_LOWER_REVERSE_MOTOR = false;
		
	// grayhill encoder polarity
	public static final boolean ALIGNED_RIGHT_UPPER_SENSOR = true;
	public static final boolean ALIGNED_RIGHT_LOWER_SENSOR = true;
	public static final boolean ALIGNED_LEFT_UPPER_SENSOR = true; 
	public static final boolean ALIGNED_LEFT_LOWER_SENSOR = true; 
	
	// PID coeffs
	private static final double kP = 1.0;
	private static final double kI = 0.0;
	private static final double kD = 0.0;
	private static final double kF = 0.0;
	
	// collector strength (%VBus - max is 1.0)
	private static final double COLLECTOR_IN_LEVEL = -0.75;
	private static final double COLLECTOR_OUT_LEVEL = 0.75;

	// teleop lift strength (%VBus - max is 1.0)
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
            
		// create and initialize collector motors (open-loop)
		leftCollectorMotor = configureMotor(HardwareIDs.LEFT_COLLECTOR_TALON_ID, LEFT_COLLECTOR_REVERSE_MOTOR);
		rightCollectorMotor = configureMotor(HardwareIDs.RIGHT_COLLECTOR_TALON_ID, RIGHT_COLLECTOR_REVERSE_MOTOR);

		// create and initialize lift motors (closed-loop)
		leftLowerLiftMotor = configureMotor(HardwareIDs.LEFT_LOWER_LIFT_TALON_ID, LEFT_LOWER_REVERSE_MOTOR, ALIGNED_LEFT_LOWER_SENSOR, kP, kI, kD, kF);
		rightLowerLiftMotor = configureMotor(HardwareIDs.RIGHT_LOWER_LIFT_TALON_ID, RIGHT_LOWER_REVERSE_MOTOR, ALIGNED_RIGHT_LOWER_SENSOR, kP, kI, kD, kF);
		leftUpperLiftMotor = configureMotor(HardwareIDs.LEFT_UPPER_LIFT_TALON_ID, LEFT_UPPER_REVERSE_MOTOR, ALIGNED_LEFT_UPPER_SENSOR, kP, kI, kD, kF);
		rightUpperLiftMotor = configureMotor(HardwareIDs.RIGHT_UPPER_LIFT_TALON_ID, RIGHT_UPPER_REVERSE_MOTOR, ALIGNED_RIGHT_UPPER_SENSOR, kP, kI, kD, kF);
				
		// make sure all motors are off
		resetMotors();
		
		// reset position of encoders
		resetPos();
		
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
    	
    // open-loop motor configuration
    private static TalonSRX configureMotor(int talonID, boolean revMotor)
    {
    	TalonSRX _talon;
    	_talon = new TalonSRX(talonID);
    	_talon.setInverted(revMotor);
    	
    	_talon.setNeutralMode(NeutralMode.Brake);
   	
    	return _talon;
    }
    
    // closed-loop motor configuration
    private static TalonSRX configureMotor(int talonID, boolean revMotor, boolean alignSensor,
    									double pCoeff, double iCoeff, double dCoeff, double fCoeff)
    {
    	TalonSRX _talon;
    	_talon = new TalonSRX(talonID);
    	_talon.setInverted(revMotor);
    	
    	_talon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, PIDLOOP_IDX, TIMEOUT_MS);
    	_talon.setSensorPhase(alignSensor); 
    	
    	_talon.selectProfileSlot(PROFILE_SLOT, PIDLOOP_IDX);
    	_talon.config_kP(PROFILE_SLOT, pCoeff, TIMEOUT_MS);
    	_talon.config_kI(PROFILE_SLOT, iCoeff, TIMEOUT_MS);
    	_talon.config_kD(PROFILE_SLOT, dCoeff, TIMEOUT_MS);
    	_talon.config_kF(PROFILE_SLOT, fCoeff, TIMEOUT_MS);
    	_talon.configMotionCruiseVelocity(0, TIMEOUT_MS);
    	_talon.configMotionAcceleration(0, TIMEOUT_MS);
    	_talon.setSelectedSensorPosition(0, PIDLOOP_IDX, TIMEOUT_MS);
 
    	_talon.setNeutralMode(NeutralMode.Brake);

    	return _talon;
    }
    
	public static void resetMotors()
	{		
		leftCollectorMotor.set(ControlMode.PercentOutput, 0);
		rightCollectorMotor.set(ControlMode.PercentOutput, 0);	
		
		leftLowerLiftMotor.set(ControlMode.PercentOutput, 0);
		leftUpperLiftMotor.set(ControlMode.PercentOutput, 0);
		rightLowerLiftMotor.set(ControlMode.PercentOutput, 0);
		rightUpperLiftMotor.set(ControlMode.PercentOutput, 0);
		
	}
	
	// resets the position encoders on lift motors
	// should be called once only, during power up  (when lift is in base state)
	public static void resetPos()
	{		
		// reset left and right lift motor encoder pulses to zero
		leftLowerLiftMotor.setSelectedSensorPosition(0, PIDLOOP_IDX, TIMEOUT_MS);
		leftUpperLiftMotor.setSelectedSensorPosition(0, PIDLOOP_IDX, TIMEOUT_MS);
		rightLowerLiftMotor.setSelectedSensorPosition(0, PIDLOOP_IDX, TIMEOUT_MS);
		rightUpperLiftMotor.setSelectedSensorPosition(0, PIDLOOP_IDX, TIMEOUT_MS);
	}	 	
	
	/************************* flipper & collector control functions **********************************/
	
	public static void flipperUp()
	{
		flipperSolenoid.set(DoubleSolenoid.Value.kForward);		
	}
	
	public static void flipperDown()
	{
		flipperSolenoid.set(DoubleSolenoid.Value.kReverse);		
	}
	
	public static void depositCube()
	{
		InputOutputComm.putDouble(InputOutputComm.LogTable.kMainLog,"CubeMgmt/CollectorLevel", COLLECTOR_OUT_LEVEL);
		leftCollectorMotor.set(ControlMode.PercentOutput, COLLECTOR_OUT_LEVEL);
		rightCollectorMotor.set(ControlMode.PercentOutput, COLLECTOR_OUT_LEVEL);		
	}
	
	/************************* lift control functions **********************************/
		
	public static void goToHeight(int level)
	{		
		int upperTargetPulses = liftLevelPulses[level][0];
		int lowerTargetPulses = liftLevelPulses[level][1];

		String posStr = String.format("%d", upperTargetPulses);
		InputOutputComm.putString(InputOutputComm.LogTable.kMainLog,"Auto/UpperLiftTargetPos", posStr);
		posStr = String.format("%d", lowerTargetPulses);
		InputOutputComm.putString(InputOutputComm.LogTable.kMainLog,"Auto/LowerLiftTargetPos", posStr);

        // configure lower lift motors
		configureMotionMagic(leftLowerLiftMotor, lowerTargetPulses);
		configureMotionMagic(rightLowerLiftMotor, lowerTargetPulses);
		
        // configure upper lift motors
		configureMotionMagic(leftUpperLiftMotor, upperTargetPulses);
		configureMotionMagic(rightUpperLiftMotor, upperTargetPulses);

	}
	
	private static void configureMotionMagic(TalonSRX _talon, int targetPulses)
	{
		_talon.configMotionCruiseVelocity(liftMotorVelocity, TIMEOUT_MS);
		_talon.configMotionAcceleration(liftMotorAccel, TIMEOUT_MS);
		_talon.set(ControlMode.MotionMagic, targetPulses);
	}
	
	/************************* UI input functions **********************************/
	
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

	public static void autoStop() {
		resetMotors();
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
	
	public static int getLiftPos(boolean upperSelected) {
		
		// Encoders now read only raw encoder values - convert raw to inches directly
		int rightUpperPos = rightUpperLiftMotor.getSelectedSensorPosition(0);
		int leftUpperPos = leftUpperLiftMotor.getSelectedSensorPosition(0);
		int rightLowerPos = rightLowerLiftMotor.getSelectedSensorPosition(0);
		int leftLowerPos = leftLowerLiftMotor.getSelectedSensorPosition(0);
				
		String posStr = String.format("%d", rightUpperPos);
		InputOutputComm.putString(InputOutputComm.LogTable.kMainLog,"Auto/LiftUpperRight", posStr);
		posStr = String.format("%d", leftUpperPos);
		InputOutputComm.putString(InputOutputComm.LogTable.kMainLog,"Auto/LiftUpperLeft", posStr);
		posStr = String.format("%d", rightLowerPos);
		InputOutputComm.putString(InputOutputComm.LogTable.kMainLog,"Auto/LiftLowerRight", posStr);
		posStr = String.format("%d", leftLowerPos);
		InputOutputComm.putString(InputOutputComm.LogTable.kMainLog,"Auto/LiftLowerLeft", posStr);
		
		if (upperSelected)
			return rightUpperPos;
		else
			return rightLowerPos;
	}
	
}
