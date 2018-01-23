package Systems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import NetworkComm.InputOutputComm;
import Utility.HardwareIDs;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;

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
			
	// grayhill encoder polarity
	public static final boolean ALIGNED_SENSOR = true; 

	// PID coeffs
	private static final double kP = 7.0;
	private static final double kI = 0.0;
	private static final double kD = 0.0;
	private static final double kF = 0.0;
	
	// collector strength (%VBus - max is 1.0)
	private static final double COLLECTOR_IN_STRENGTH = 1.00;
	private static final double COLLECTOR_OUT_STRENGTH = -1.00;
	private static final boolean LEFT_COLLECTOR_INVERTED = true;
	private static final boolean RIGHT_COLLECTOR_INVERTED = false;

	// teleop lift strength (%VBus - max is 1.0)
	private static final double LIFT_UP_STRENGTH = -0.25;
	private static final double LIFT_DOWN_STRENGTH = 0.25;
	private static final double LIFT_MOTOR_FACTOR = 1.0;
	private static final double LIFT_MOTOR_DEAD_ZONE = 0.1;
	
	// control dead zone threshold
	private static final double DEAD_ZONE_THRESHOLD = 0.05;
	
	// idle time beyond which lift brake is applied
	private static final double LIFT_IDLE_BRAKE_SECONDS = 0.25;
	private static double liftIdleTimerStart = 0;
		    					
	// collector intake motors
	private static Spark leftCollectorMotor, rightCollectorMotor; 
	
	// lift motors
	private static TalonSRX upperLiftMotor, lowerLiftMotor;
		
	// pneumatics objects
	private static Compressor compress;
	
	private static DoubleSolenoid flipperSolenoid;
	private static boolean flipperUp;
	
	private static DoubleSolenoid clampSolenoid;
	private static boolean clampOn;
	private static double clampStartTimer = 0;
	private static final double CLAMP_LIMIT_SEC = 0.25;
	
	private static DoubleSolenoid liftBrakeSolenoid;
	private static boolean liftBrakeOn;
	
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
		/*
		compress = new Compressor(HardwareIDs.PCM_ID);
		flipperSolenoid = new DoubleSolenoid(HardwareIDs.PCM_ID, HardwareIDs.FLIPPER_UP_SOLENOID, HardwareIDs.FLIPPER_DOWN_SOLENOID);
		clampSolenoid = new DoubleSolenoid(HardwareIDs.PCM_ID, HardwareIDs.CLAMP_ON_SOLENOID, HardwareIDs.CLAMP_OFF_SOLENOID);
		liftBrakeSolenoid = new DoubleSolenoid(HardwareIDs.PCM_ID, HardwareIDs.BRAKE_ON_SOLENOID, HardwareIDs.BRAKE_OFF_SOLENOID);
        */
		
		// create and initialize collector motors (open-loop)
		leftCollectorMotor = new Spark(HardwareIDs.LEFT_COLLECTOR_PWM_ID);
		leftCollectorMotor.setInverted(LEFT_COLLECTOR_INVERTED);
		rightCollectorMotor = new Spark(HardwareIDs.RIGHT_COLLECTOR_PWM_ID);
		rightCollectorMotor.setInverted(RIGHT_COLLECTOR_INVERTED);

		/*
		// create and initialize upper lift motor (closed-loop)
		upperLiftMotor = configureMotor(HardwareIDs.UPPER_LIFT_TALON_ID, UPPER_REVERSE_MOTOR, ALIGNED_SENSOR, kP, kI, kD, kF);

		// create and initialize lower lift motor (follows upper motor)
		lowerLiftMotor = configureMotor(HardwareIDs.LOWER_LIFT_TALON_ID, LOWER_REVERSE_MOTOR, HardwareIDs.UPPER_LIFT_TALON_ID);
		*/	
				
		// make sure all motors are off
		resetMotors();
		
		// reset position of encoders
		resetPos();
		
		// lift up flipper
		flipperUp();
		
		// clamp on
		clampOn();
		clampStartTimer = RobotController.getFPGATime();
		
		// turn on brake
		liftBrakeOn();
		
		gamepad = new Joystick(HardwareIDs.GAMEPAD_ID);
		
		initialized = true;
	}
	
	public static void resetMotors()
	{		
		leftCollectorMotor.set(0);
		rightCollectorMotor.set(0);	
		
		// reset upper lift motor (lower lift follows)
		//upperLiftMotor.set(ControlMode.PercentOutput, 0);
	}
	
	// resets the position encoders on lift motors
	// should be called once only, during power up  (when lift is in base state)
	public static void resetPos()
	{		
		// reset upper lift motor encoder pulses to zero
		//upperLiftMotor.setSelectedSensorPosition(0, PIDLOOP_IDX, TIMEOUT_MS);
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
    	
    	// set up sensor
    	_talon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, PIDLOOP_IDX, TIMEOUT_MS);
    	_talon.setSensorPhase(alignSensor); 
    	
    	// set up limit switches
		//_talon.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
		//_talon.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
   	
    	// set up closed loop control
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
    
	
	/************************* flipper, lift brake & collector control functions **********************************/
	
	public static void flipperUp()
	{
		flipperUp = true;
		//flipperSolenoid.set(DoubleSolenoid.Value.kForward);	
		InputOutputComm.putBoolean(InputOutputComm.LogTable.kMainLog,"CubeMgmt/Flipper", flipperUp);
	}
	
	public static void flipperDown()
	{
		flipperUp = false;
		//flipperSolenoid.set(DoubleSolenoid.Value.kReverse);		
		InputOutputComm.putBoolean(InputOutputComm.LogTable.kMainLog,"CubeMgmt/Flipper", flipperUp);
	}

	public static void clampOn()
	{
		clampOn = true;
		//clampSolenoid.set(DoubleSolenoid.Value.kForward);		
		InputOutputComm.putBoolean(InputOutputComm.LogTable.kMainLog,"CubeMgmt/Clamp", clampOn);
	}

	public static void clampOff()
	{
		clampOn = false;
		//clampSolenoid.set(DoubleSolenoid.Value.kReverse);		
		InputOutputComm.putBoolean(InputOutputComm.LogTable.kMainLog,"CubeMgmt/Clamp", clampOn);
	}

	public static void liftBrakeOn()
	{
		liftBrakeOn = true;
		//liftBrakeSolenoid.set(DoubleSolenoid.Value.kForward);		
		InputOutputComm.putBoolean(InputOutputComm.LogTable.kMainLog,"CubeMgmt/LiftBrake", liftBrakeOn);
	}

	public static void liftBrakeOff()
	{
		liftBrakeOn = false;
		//liftBrakeSolenoid.set(DoubleSolenoid.Value.kReverse);		
		InputOutputComm.putBoolean(InputOutputComm.LogTable.kMainLog,"CubeMgmt/LiftBrake", liftBrakeOn);
	}

	public static void depositCube()
	{
		InputOutputComm.putDouble(InputOutputComm.LogTable.kMainLog,"CubeMgmt/CollectorLevel", COLLECTOR_OUT_STRENGTH);
		leftCollectorMotor.set(COLLECTOR_OUT_STRENGTH);
		rightCollectorMotor.set(COLLECTOR_OUT_STRENGTH);
			
	}
	
	/************************* lift control functions **********************************/
		
	public static void goToHeight(int level)
	{		
		int targetPulses = liftLevelPulses[level];

		String posStr = String.format("%d", targetPulses);
		InputOutputComm.putString(InputOutputComm.LogTable.kMainLog,"Auto/liftTargetPulses", posStr);
		
        // configure upper lift motor (lower lift motor follows)
		//configureMotionMagic(upperLiftMotor, targetPulses);
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
		double collectorStrength = gamepad.getRawAxis(HardwareIDs.COLLECTOR_IN_AXIS);
		if (Math.abs(collectorStrength) > DEAD_ZONE_THRESHOLD)
			collectorStrength = COLLECTOR_IN_STRENGTH;
		else if (gamepad.getRawButton(HardwareIDs.COLLECTOR_OUT_BUTTON))
			collectorStrength = COLLECTOR_OUT_STRENGTH;
		else
			collectorStrength = 0.0;
		InputOutputComm.putDouble(InputOutputComm.LogTable.kMainLog,"CubeMgmt/CollectorStrength", collectorStrength);
		leftCollectorMotor.set(collectorStrength);
		rightCollectorMotor.set(collectorStrength);
	}
	
	private static void checkClampControls() {

		// not enough time between clamp control events
		if ((RobotController.getFPGATime() - clampStartTimer) < CLAMP_LIMIT_SEC)
			return;

		if (gamepad.getRawButton(HardwareIDs.CLAMP_TOGGLE_BUTTON)) {
			if (clampOn)
				clampOff();
			else
				clampOn();
		}
		
		// reset clamp timer
		clampStartTimer = RobotController.getFPGATime();
	}
	
	private static void checkLiftControls() {
		
		// cube lift control
		double liftStrength = gamepad.getRawAxis(HardwareIDs.LIFT_MOTOR_AXIS);
				
		// convert joystick value into motor speed value
		if (Math.abs(liftStrength) >= LIFT_MOTOR_DEAD_ZONE)
			liftStrength *= LIFT_MOTOR_FACTOR; 
		else 
			liftStrength = 0.0;
			
		// if directed lift gain is zero (idle)
		if (liftStrength == 0)
		{
			// if brake is off and (lift idle time > idle threshold)
			if ((!liftBrakeOn) && (
					(RobotController.getFPGATime() - liftIdleTimerStart) > LIFT_IDLE_BRAKE_SECONDS)) {
				// apply brake
				liftBrakeOn();			
			}
		}
		else {
			
			// if brake is on and gain is non-zero (not idle)
			if (liftBrakeOn)
			{
				// turn off brake
				liftBrakeOff();
				
				// wait a brief moment for brake to mechanically deactivate
				Timer.delay(0.2);
			}	
			
			// reset lift idle timer start
			liftIdleTimerStart = RobotController.getFPGATime();
		}
	
		// apply lift motor gain value
		InputOutputComm.putDouble(InputOutputComm.LogTable.kMainLog,"CubeMgmt/LiftStrength", liftStrength);
		//upperLiftMotor.set(ControlMode.PercentOutput, liftStrength);
	}
	
	private static void checkFlipperControls() {
	
		double flipperInput = gamepad.getRawAxis(HardwareIDs.FLIPPER_DOWN_AXIS);
		if ((Math.abs(flipperInput) > DEAD_ZONE_THRESHOLD) && (flipperUp)) {
			flipperDown();	
		}
		else if (gamepad.getRawButton(HardwareIDs.FLIPPER_UP_BUTTON) && (!flipperUp)) {
			flipperUp();
		}			
	}
	
	public static void autoInit() {				
		resetMotors();
		flipperUp();
		liftBrakeOn();		
	}

	public static void autoStop() {
		resetMotors();
	}
	
	public static void teleopInit() {
				
		resetMotors();
		flipperUp();
		liftBrakeOn();
				
		liftIdleTimerStart = RobotController.getFPGATime();      
	}
	
	public static void teleopPeriodic() {
		
		checkCollectorControls();
		checkClampControls();
		checkLiftControls();
		checkFlipperControls();
	}
	
	public static int getLiftPos() {
		
		// Encoders now read only raw encoder values - convert raw to inches directly
		//int upperPulses = upperLiftMotor.getSelectedSensorPosition(0);
		int upperPulses = 0;
				
		String posStr = String.format("%d", upperPulses);
		InputOutputComm.putString(InputOutputComm.LogTable.kMainLog,"Auto/LiftUpper", posStr);
		
		return upperPulses;
	}
	
}
