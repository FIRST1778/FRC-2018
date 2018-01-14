package Systems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import NetworkComm.InputOutputComm;
import Utility.HardwareIDs;

//Chill Out 1778 class for controlling the drivetrain during auto

public class AutoDriveAssembly {
		
	private static boolean initialized = false;
	private static final int TIMEOUT_MS = 0;  // set to zero if skipping confirmation
	private static final int PIDLOOP_IDX = 0;  // set to zero if primary loop
	private static final int PROFILE_SLOT = 0;
	
	public static void initialize() {
		
		if (initialized)
			return;
		
		//ioComm = InputOutputComm.GetInstance();
		NavXSensor.initialize();
		TurnController.initialize();
		
		// instantiate motion profile motor control objects
        mFrontLeft = new TalonSRX(HardwareIDs.LEFT_FRONT_TALON_ID);
        mFrontLeft.setInverted(LEFT_REVERSE_MOTOR);  // left motor PID polarity (magic motion mode only)
        
        //mBackLeft = new TalonSRX(HardwareIDs.LEFT_REAR_TALON_ID);

		mFrontRight = new TalonSRX(HardwareIDs.RIGHT_FRONT_TALON_ID);
		mFrontRight.setInverted(RIGHT_REVERSE_MOTOR);  // right motor PID polarity (magic motion mode only)

		//mBackRight = new TalonSRX(HardwareIDs.RIGHT_REAR_TALON_ID);
        
        // configure left front motor encoder and PID
        mFrontLeft.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, PIDLOOP_IDX, TIMEOUT_MS);
        mFrontLeft.setSensorPhase(ALIGNED_LEFT_SENSOR);   // left motor encoder polarity
        		
        mFrontLeft.selectProfileSlot(PROFILE_SLOT, PIDLOOP_IDX);
        mFrontLeft.config_kP(PROFILE_SLOT, P_COEFF, TIMEOUT_MS);
        mFrontLeft.config_kI(PROFILE_SLOT, I_COEFF, TIMEOUT_MS);
        mFrontLeft.config_kD(PROFILE_SLOT, D_COEFF, TIMEOUT_MS);
        mFrontLeft.config_kF(PROFILE_SLOT, F_COEFF, TIMEOUT_MS);
        mFrontLeft.configMotionCruiseVelocity(0, TIMEOUT_MS);
        mFrontLeft.configMotionAcceleration(0, TIMEOUT_MS);
        mFrontLeft.setSelectedSensorPosition(0, PIDLOOP_IDX, TIMEOUT_MS);
        
        // configure right front motor encoder and PID
        mFrontRight.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, PIDLOOP_IDX, TIMEOUT_MS);
        mFrontRight.setSensorPhase(ALIGNED_RIGHT_SENSOR);   // right motor encoder polarity
 
        mFrontRight.selectProfileSlot(PROFILE_SLOT, PIDLOOP_IDX);
        mFrontRight.config_kP(PROFILE_SLOT, P_COEFF, TIMEOUT_MS);
        mFrontRight.config_kI(PROFILE_SLOT, I_COEFF, TIMEOUT_MS);
        mFrontRight.config_kD(PROFILE_SLOT, D_COEFF, TIMEOUT_MS);
        mFrontRight.config_kF(PROFILE_SLOT, F_COEFF, TIMEOUT_MS);
        mFrontRight.configMotionCruiseVelocity(0, TIMEOUT_MS);
        mFrontRight.configMotionAcceleration(0, TIMEOUT_MS);
        mFrontRight.setSelectedSensorPosition(0, PIDLOOP_IDX, TIMEOUT_MS);
        
        initialized = true;
	}
					
	private static final double AUTO_DRIVE_ANGLE_CORRECT_COEFF = 0.02;
	private static final double GYRO_CORRECT_COEFF = 0.03;
			
	// smart controllers (motion profiling)
	private static TalonSRX mFrontLeft, mFrontRight;
	//private static TalonSRX mBackLeft, mBackRight;
		
	// used as angle baseline (if we don't reset gyro)
	private static double initialAngle = 0.0;
	
	// original encoder variables
	//private static final int ENCODER_PULSES_PER_REV = 250;  // E4P-250  - on the proto bot front motors
	//private static final double INCHES_PER_REV = (6 * 3.14159);   // 6-in diameter wheel

	// motor polarity
	public static final boolean RIGHT_REVERSE_MOTOR = true;     // comp-bot motor polarity - right
	public static final boolean LEFT_REVERSE_MOTOR = false;		// comp-bot motor polarity - left
		
	// grayhill encoder polarity
	public static final boolean ALIGNED_RIGHT_SENSOR = true;	// encoder polarity - right
	public static final boolean ALIGNED_LEFT_SENSOR = true;    // encoder polarity - left
	
	//public static final int ENCODER_PULSES_PER_REV = 256;  // 63R  - on the competition bot front motors
	public static final int ENCODER_PULSES_PER_REV = 256*4;  // 63R  - on the competition bot front motors

	//public static final double INCHES_PER_REV = (6 * 3.14159);   // 6-in diameter wheel (theoretical)
	public static final double INCHES_PER_REV = (5.9 * 3.14159);   // 5.9-in diameter wheel (worn)
			
	public static final double INCHES_PER_ENCODER_PULSE = INCHES_PER_REV/ENCODER_PULSES_PER_REV;
	public static final double RPM_TO_UNIT_PER_100MS = ENCODER_PULSES_PER_REV/(60*10);
	
	// PIDF values - proto.bot - initial 
	private static final double P_COEFF = 7.0;
	private static final double I_COEFF = 0.0;  // Integral not needed for closed loop position control
	private static final double D_COEFF = 0.0;
	private static final double F_COEFF = 0.0;  // Feedforward not used for closed loop position control
		
	private static void resetMotors()
	{
		// disable brake mode (all motors on coast)
		mFrontLeft.setNeutralMode(NeutralMode.Coast);
		mFrontRight.setNeutralMode(NeutralMode.Coast);
		//mBackLeft.setNeutralMode(NeutralMode.Coast);
		//mBackRight.setNeutralMode(NeutralMode.Coast);
		
		//ioComm.putBoolean(InputOutputComm.LogTable.kMainLog,"Auto/BrakeMode", false);
		
		// turn all motors to zero power
		mFrontLeft.set(ControlMode.PercentOutput,0.0);
		mFrontRight.set(ControlMode.PercentOutput,0.0);		
		//mBackLeft.set(ControlMode.PercentOutput,0.0);
		//mBackRight.set(ControlMode.PercentOutput,0.0);
		
	}
		
	private static void configureMotorsVbus() 
	{		
		// for auto - brake mode enabled
		mFrontLeft.setNeutralMode(NeutralMode.Brake);
		mFrontRight.setNeutralMode(NeutralMode.Brake);
		//mBackLeft.setNeutralMode(NeutralMode.Brake);
		//mBackRight.setNeutralMode(NeutralMode.Brake);
		
		//ioComm.putBoolean(InputOutputComm.LogTable.kMainLog,"Auto/BrakeMode", true);
		
		// turn all motors to zero power		
		mFrontLeft.set(ControlMode.PercentOutput,0);
		mFrontRight.set(ControlMode.PercentOutput,0);		
		//mBackLeft.set(ControlMode.PercentOutput,0);
		//mBackRight.set(ControlMode.PercentOutput,0);
			
	}

	private static void configureMotorsMagic() 
	{		
		// for auto - brake mode NOT enabled
		mFrontLeft.setNeutralMode(NeutralMode.Coast);
		mFrontRight.setNeutralMode(NeutralMode.Coast);
		//mBackLeft.setNeutralMode(NeutralMode.Coast);
		//mBackRight.setNeutralMode(NeutralMode.Coast);
		
		//ioComm.putBoolean(InputOutputComm.LogTable.kMainLog,"Auto/BrakeMode", false);
		
        // configure left and right front motors for magic motion (closed-loop position control)
		mFrontRight.set(ControlMode.MotionMagic, 0);
		mFrontLeft.set(ControlMode.MotionMagic, 0);
        //mBackRight.follow(mFrontRight);
        //mBackLeft.follow(mFrontLeft);		
	}

	public static void resetPos()
	{		
		// reset front left and right encoder pulses to zero
		mFrontLeft.setSelectedSensorPosition(0, PIDLOOP_IDX, TIMEOUT_MS);
		mFrontRight.setSelectedSensorPosition(0, PIDLOOP_IDX, TIMEOUT_MS);
	}	 	        

	public static double getDistanceInches() {
				
		// Encoders now read only raw encoder values - convert raw to inches directly
		double rightPos = mFrontRight.getSelectedSensorPosition(0)*INCHES_PER_ENCODER_PULSE;
		double leftPos = mFrontLeft.getSelectedSensorPosition(0)*INCHES_PER_ENCODER_PULSE;
				
		String posStr = String.format("%.2f", rightPos);
		InputOutputComm.putString(InputOutputComm.LogTable.kMainLog,"Auto/EncoderRight", posStr);
		posStr = String.format("%.2f", leftPos);
		InputOutputComm.putString(InputOutputComm.LogTable.kMainLog,"Auto/EncoderLeft", posStr);
		
		return rightPos;
	}
	
	public static void autoInit(boolean resetGyro, double headingDeg, boolean magicMotion) {
				
		if (resetGyro) {
			NavXSensor.reset();
			initialAngle = 0.0;
		}
		else
			//initialAngle = NavXSensor.getAngle();
			initialAngle = headingDeg;				// target heading if not resetting gyro

		if (magicMotion)
			// configure motors for magic motion
			configureMotorsMagic();
		else
			// configure motors for normal VBus control
			configureMotorsVbus();
		
		// set current position to zero
		resetPos();
   	}
					
	public static void autoGyroStraight(double speed) {
		// autonomous operation of drive straight - uses gyro
		
		double gyroAngle = NavXSensor.getAngle();
		
		// subtract the initial angle offset, if any
		gyroAngle -= initialAngle;
		
		// calculate speed adjustment for left and right sides (negative sign added as feedback)
		double driveAngle = -gyroAngle * AUTO_DRIVE_ANGLE_CORRECT_COEFF;
				
		double leftSpeed = speed+driveAngle;		
		double rightSpeed = speed-driveAngle;
				
		// adjust speed of left and right sides
		drive(leftSpeed, rightSpeed);		 
	}
	
	public static void autoMagicStraight(double targetPosInches, int speedRpm, int accelRpm) {
		
		int nativeUnitsPer100ms = (int) ((double)speedRpm * RPM_TO_UNIT_PER_100MS);
		int accelNativeUnits = (int) ((double)accelRpm * RPM_TO_UNIT_PER_100MS);

        // left front drive straight - uses motion magic
		mFrontLeft.configMotionCruiseVelocity(nativeUnitsPer100ms, TIMEOUT_MS);
		mFrontLeft.configMotionAcceleration(accelNativeUnits, TIMEOUT_MS);
		mFrontLeft.set(ControlMode.MotionMagic, targetPosInches/INCHES_PER_ENCODER_PULSE);
		
        // right front drive straight - uses motion magic
        mFrontRight.configMotionCruiseVelocity(nativeUnitsPer100ms, TIMEOUT_MS);
        mFrontRight.configMotionAcceleration(accelNativeUnits, TIMEOUT_MS);
        mFrontRight.set(ControlMode.MotionMagic, targetPosInches/INCHES_PER_ENCODER_PULSE);
		
		// left and right back motors are following front motors
	}
	
	public static void autoMagicTurn(double targetPosInchesLeft, double targetPosInchesRight, int speedRpm, int accelRpm) {

		int nativeUnitsPer100ms = (int) ((double)speedRpm * RPM_TO_UNIT_PER_100MS);
		int accelNativeUnits = (int) ((double)accelRpm * RPM_TO_UNIT_PER_100MS);
		
        // left front drive straight - uses motion magic
		mFrontLeft.configMotionCruiseVelocity(nativeUnitsPer100ms, TIMEOUT_MS);
		mFrontLeft.configMotionAcceleration(accelNativeUnits, TIMEOUT_MS);
		mFrontLeft.set(ControlMode.MotionMagic, targetPosInchesLeft/INCHES_PER_ENCODER_PULSE);
		
        // right front drive straight - uses motion magic
        mFrontRight.configMotionCruiseVelocity(nativeUnitsPer100ms, TIMEOUT_MS);
        mFrontRight.configMotionAcceleration(accelNativeUnits, TIMEOUT_MS);
        mFrontRight.set(ControlMode.MotionMagic, targetPosInchesRight/INCHES_PER_ENCODER_PULSE);
				
		// left and right back motors are following front motors
	}
	
	public static void autoStop() {
		resetMotors();
	}
	
	// auto PID turn methods
	public static void autoPidTurnStart(double angleDeg, double speed) {
		resetMotors();
		TurnController.setAngle(angleDeg, speed);
		TurnController.enable();
	}
	
	public static void autoPidTurnProcess() {
		double leftValue = TurnController.getLeft();
		double rightValue = TurnController.getRight();
		drive(leftValue,rightValue);
	}
	
	public static void autoPidTurnStop() {
		TurnController.disable();
		resetMotors();
	}
		
	public static void teleopInit() {
	}
	
	public static void teleopPeriodic() {	
	}
	
	public static void disabledInit( )  {
		resetMotors();
	}
	
	// CORE DRIVE METHOD
	// Assumes parameters are PercentVbus (0.0 to 1.0)
	public static void drive(double leftValue, double rightValue) {
		
		String leftSpeedStr = String.format("%.2f", leftValue);
		String rightSpeedStr = String.format("%.2f", rightValue);
		String myString2 = new String("leftSpeed = " + leftSpeedStr + " rightSpeed = " + rightSpeedStr);
		//System.out.println(myString2);
		//ioComm.putString(InputOutputComm.LogTable.kMainLog,"Auto/AutoDrive", myString2);

		// set motor values directly
		mFrontLeft.set(ControlMode.PercentOutput, leftValue);
		mFrontRight.set(ControlMode.PercentOutput, rightValue);
		//mBackLeft.set(ControlMode.PercentOutput, leftValue);
		//mBackRight.set(ControlMode.PercentOutput, rightValue);		
	}
	
	public static void driveDirection(double angle, double speed) {
		double gyroAngle = NavXSensor.getAngle();	
		double driveAngle = (angle-gyroAngle)*GYRO_CORRECT_COEFF;
		drive(driveAngle+speed, -driveAngle+speed);
	}
	
	public static void turnToDirection(double angle, double power) {
		double gyroAngle = NavXSensor.getAngle();
		double driveAngle = (angle-gyroAngle)*(1/360)*power;
		drive(driveAngle, -driveAngle);
	}
	
	public static void driveForward(double forwardVel) {
		drive(forwardVel, forwardVel);
	}
	
	public static void rotate(double angularVel) {
		drive(angularVel, -angularVel);
	}
	
	public static void driveVelocity(double forwardVel, double angularVel) {
		drive((forwardVel+angularVel)/2.0,(forwardVel-angularVel)/2.0);
	}
		
	//Turn methods
	//===================================================
	public static void rotateLeft(double speed) {		
		drive(-speed, speed);
	}

	public static void rotateRight(double speed) {
		drive(speed, -speed);
	}
}
