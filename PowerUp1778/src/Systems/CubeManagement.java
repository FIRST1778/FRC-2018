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
import edu.wpi.first.wpilibj.Spark;

public class CubeManagement {
	
	private static boolean initialized = false;
	private static final int TIMEOUT_MS = 0;  // set to zero if skipping confirmation
	private static final int PIDLOOP_IDX = 0;  // set to zero if primary loop
	private static final int PROFILE_SLOT = 0;

	// auto lift levels
	public static final int BASE_LEVEL = 0;
	public static final int SWITCH_LEVEL = 1;
	public static final int SCALE_LEVEL = 2;
	
	private static final int liftLevelPulses[] = {0, 100, 200};  // number of encoder pulses for each level  {upper, lower}
	private static final int speedRpm = 900;
	private static final int accelRpm = 450;
	
	// motor polarity
	public static final boolean RIGHT_COLLECTOR_REVERSE_MOTOR = false; 
	public static final boolean LEFT_COLLECTOR_REVERSE_MOTOR = false; 
	
	public static final boolean UPPER_REVERSE_MOTOR = false; 
	public static final boolean LOWER_REVERSE_MOTOR = false; 
		
	// grayhill encoder polarity
	public static final boolean ALIGNED_SENSOR = true; 

	// PID coeffs
	private static final double kP = 7.0;
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
		    					
	// collector intake motors
	private static Spark leftCollectorMotor, rightCollectorMotor; 
	
	// lift motors
	private static TalonSRX upperLiftMotor, lowerLiftMotor;
		
	// pneumatics objects
	private static Compressor compress;
	private static DoubleSolenoid flipperSolenoid;
	
	private static Joystick gamepad;
		
	// wait 0.25 s between button pushes on shooter
    private static final int TRIGGER_CYCLE_WAIT_US = 250000;
    private static double initTriggerTime;

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
		leftCollectorMotor = new Spark(HardwareIDs.LEFT_COLLECTOR_PWM_ID);
		rightCollectorMotor = new Spark(HardwareIDs.RIGHT_COLLECTOR_PWM_ID);

		// create and initialize upper lift motor (closed-loop)
		upperLiftMotor = configureMotor(HardwareIDs.UPPER_LIFT_TALON_ID, UPPER_REVERSE_MOTOR, ALIGNED_SENSOR, kP, kI, kD, kF);

		// create and initialize lower lift motor (follows upper motor)
		lowerLiftMotor = configureMotor(HardwareIDs.LOWER_LIFT_TALON_ID, LOWER_REVERSE_MOTOR, HardwareIDs.UPPER_LIFT_TALON_ID);
				
		// make sure all motors are off
		resetMotors();
		
		// reset position of encoders
		resetPos();
		
		gamepad = new Joystick(HardwareIDs.GAMEPAD_ID);
		
		initialized = true;
	}
	
	public static void resetMotors()
	{		
		leftCollectorMotor.set(0);
		rightCollectorMotor.set(0);	
		
		// reset upper lift motor (lower lift follows)
		upperLiftMotor.set(ControlMode.PercentOutput, 0);
	}
	
	// resets the position encoders on lift motors
	// should be called once only, during power up  (when lift is in base state)
	public static void resetPos()
	{		
		// reset upper lift motor encoder pulses to zero
		upperLiftMotor.setSelectedSensorPosition(0, PIDLOOP_IDX, TIMEOUT_MS);
	}	 	

    // open-loop motor configuration (and possibly follower)
    private static TalonSRX configureMotor(int talonID, boolean revMotor, int talonIDToFollow)
    {
    	TalonSRX _talon;
    	_talon = new TalonSRX(talonID);
    	_talon.setInverted(revMotor);
    	
    	if (talonIDToFollow > 0)
    		_talon.set(ControlMode.Follower, (double)talonIDToFollow);
    	
    	//_talon.setNeutralMode(NeutralMode.Brake);
   	
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
 
    	//_talon.setNeutralMode(NeutralMode.Brake);

    	return _talon;
    }
    
	
	/************************* flipper & collector control functions **********************************/
	
	public static void flipperUp()
	{
		InputOutputComm.putString(InputOutputComm.LogTable.kMainLog,"CubeMgmt/Flipper", "Up");
		flipperSolenoid.set(DoubleSolenoid.Value.kForward);		
	}
	
	public static void flipperDown()
	{
		InputOutputComm.putString(InputOutputComm.LogTable.kMainLog,"CubeMgmt/Flipper", "Down");
		flipperSolenoid.set(DoubleSolenoid.Value.kReverse);		
	}
	
	public static void depositCube()
	{
		InputOutputComm.putDouble(InputOutputComm.LogTable.kMainLog,"CubeMgmt/CollectorLevel", COLLECTOR_OUT_LEVEL);
		leftCollectorMotor.set(COLLECTOR_OUT_LEVEL);
		rightCollectorMotor.set(COLLECTOR_OUT_LEVEL);
			
	}
	
	/************************* lift control functions **********************************/
		
	public static void goToHeight(int level)
	{		
		int targetPulses = liftLevelPulses[level];

		String posStr = String.format("%d", targetPulses);
		InputOutputComm.putString(InputOutputComm.LogTable.kMainLog,"Auto/liftTargetPulses", posStr);
		
        // configure upper lift motor (lower lift motor follows)
		configureMotionMagic(upperLiftMotor, targetPulses);
	}
	
	private static void configureMotionMagic(TalonSRX _talon, int targetPulses)
	{
		int nativeUnitsPer100ms = (int) ((double)speedRpm * HardwareIDs.RPM_TO_UNIT_PER_100MS);
		int accelNativeUnits = (int) ((double)accelRpm * HardwareIDs.RPM_TO_UNIT_PER_100MS);

		_talon.configMotionCruiseVelocity(nativeUnitsPer100ms, TIMEOUT_MS);
		_talon.configMotionAcceleration(accelNativeUnits, TIMEOUT_MS);
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
		leftCollectorMotor.set(collectorLevel);
		rightCollectorMotor.set(collectorLevel);
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
		upperLiftMotor.set(ControlMode.PercentOutput, liftLevel);
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
	
	public static int getLiftPos() {
		
		// Encoders now read only raw encoder values - convert raw to inches directly
		int upperPulses = upperLiftMotor.getSelectedSensorPosition(0);
				
		String posStr = String.format("%d", upperPulses);
		InputOutputComm.putString(InputOutputComm.LogTable.kMainLog,"Auto/LiftUpper", posStr);
		
		return upperPulses;
	}
	
}
