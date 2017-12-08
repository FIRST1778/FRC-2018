package Systems;

import com.ctre.phoenix.MotorControl.CAN.TalonSRX;
import com.ctre.phoenix.MotorControl.SmartMotorController.FeedbackDevice;

import NetworkComm.InputOutputComm;
import Utility.HardwareIDs;

//Chill Out 1778 class for controlling the drivetrain during auto

public class AutoDriveAssembly {
		
	private static boolean initialized = false;
	
	public static void initialize() {
		
		if (initialized)
			return;
		
		//ioComm = InputOutputComm.GetInstance();
		NavXSensor.initialize();
		TurnController.initialize();
		
		// instantiate motion profile motor control objects
        mFrontLeft = new TalonSRX(HardwareIDs.LEFT_FRONT_TALON_ID);
        mFrontLeft.reverseOutput(LEFT_REVERSE_MOTOR);	   // left motor PID polarity (magic motion mode only)
		//mBackLeft = new TalonSRX(HardwareIDs.LEFT_REAR_TALON_ID);
		//mBackLeft.reverseOutput(false);	   // left back motor feedback polarity (follower mode only)

		mFrontRight = new TalonSRX(HardwareIDs.RIGHT_FRONT_TALON_ID);
		mFrontRight.reverseOutput(RIGHT_REVERSE_MOTOR);  // right motor PID polarity (magic motion mode only)
		//mBackRight = new TalonSRX(HardwareIDs.RIGHT_REAR_TALON_ID);
		//mBackRight.reverseOutput(true);  // right back motor feedback polarity (follower mode only)
        
        // configure left front motor encoder and PID
        mFrontLeft.setFeedbackDevice(FeedbackDevice.QuadEncoder);
        //mFrontLeft.configEncoderCodesPerRev(ENCODER_PULSES_PER_REV);    // deprecated 2018
        mFrontLeft.reverseSensor(LEFT_REVERSE_SENSOR);   // left motor encoder polarity
        mFrontLeft.setProfile(0);
        mFrontLeft.setP(P_COEFF);
        mFrontLeft.setI(I_COEFF);
        mFrontLeft.setD(D_COEFF);
        mFrontLeft.setF(F_COEFF);
        mFrontLeft.setMotionMagicCruiseVelocity(0);
        mFrontLeft.setMotionMagicAcceleration(0);
        
        // configure right front motor encoder and PID
        mFrontRight.setFeedbackDevice(FeedbackDevice.QuadEncoder);
        //mFrontRight.configEncoderCodesPerRev(ENCODER_PULSES_PER_REV);    // deprecated 2018
        mFrontRight.reverseSensor(RIGHT_REVERSE_SENSOR);   // right motor encoder polarity
        mFrontRight.setProfile(0);
        mFrontRight.setP(P_COEFF);
        mFrontRight.setI(I_COEFF);
        mFrontRight.setD(D_COEFF);
        mFrontRight.setF(F_COEFF);
        mFrontRight.setMotionMagicCruiseVelocity(0);
        mFrontRight.setMotionMagicAcceleration(0);
        
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

	// grayhill encoder variables
	public static final boolean RIGHT_REVERSE_MOTOR = true;		// comp-bot PID motor polarity - right
	public static final boolean LEFT_REVERSE_MOTOR = false;		// comp-bot PID motor polarity - left
	
	//public static final boolean RIGHT_REVERSE_MOTOR = false;  // proto-bot PID motor polarity - right
	//public static final boolean LEFT_REVERSE_MOTOR = true;	// proto-bot PID motor polarity - left
	
	public static final boolean RIGHT_REVERSE_SENSOR = false;	// encoder polarity - right
	public static final boolean LEFT_REVERSE_SENSOR = true;		// encoder polarity - left
	
	//public static final int ENCODER_PULSES_PER_REV = 256;  // 63R  - on the competition bot front motors
	public static final int ENCODER_PULSES_PER_REV = 256*4;  // 63R  - on the competition bot front motors

	//public static final double INCHES_PER_REV = (6 * 3.14159);   // 6-in diameter wheel (theoretical)
	public static final double INCHES_PER_REV = (5.9 * 3.14159);   // 5.9-in diameter wheel (worn)
			
	public static final double INCHES_PER_ENCODER_PULSE = INCHES_PER_REV/ENCODER_PULSES_PER_REV;
	public static final double RPM_TO_UNIT_PER_100MS = ENCODER_PULSES_PER_REV/(60*10);
	
	// PIDF values - comp.bot version tuned 7/20/2017
	//private static final double P_COEFF = 20.0;
	//private static final double I_COEFF = 0.0;  // Integral not needed for closed loop position control
	//private static final double D_COEFF = 16.0;
	//private static final double F_COEFF = 0.0;  // Feedforward not used for closed loop position control

	// PIDF values - proto.bot - tuned 
	private static final double P_COEFF = 10.0;
	private static final double I_COEFF = 0.0;  // Integral not needed for closed loop position control
	private static final double D_COEFF = 0.0;
	private static final double F_COEFF = 0.0;  // Feedforward not used for closed loop position control
		
	private static void resetMotors()
	{
		// disable brake mode (all motors on coast)
	    mFrontLeft.enableBrakeMode(false);
		mFrontRight.enableBrakeMode(false);
		//mBackLeft.enableBrakeMode(false);
		//mBackRight.enableBrakeMode(false);
		//ioComm.putBoolean(InputOutputComm.LogTable.kMainLog,"Auto/BrakeMode", false);
		
		// reset control mode to VBus mode (% pwr) 
		mFrontLeft.changeControlMode(TalonSRX.TalonControlMode.PercentVbus);
		mFrontRight.changeControlMode(TalonSRX.TalonControlMode.PercentVbus);
		//mBackLeft.changeControlMode(TalonSRX.TalonControlMode.PercentVbus);
		//mBackRight.changeControlMode(TalonSRX.TalonControlMode.PercentVbus);

		// turn all motors to zero power
		mFrontLeft.set(0);
		mFrontRight.set(0);
		//mBackLeft.set(0);
		//mBackRight.set(0);
		
	}
		
	private static void configureMotorsVbus() 
	{		
		// for auto - brake mode enabled
	    mFrontLeft.enableBrakeMode(true);
		mFrontRight.enableBrakeMode(true);
		//mBackLeft.enableBrakeMode(true);
		//mBackRight.enableBrakeMode(true);
		
		//ioComm.putBoolean(InputOutputComm.LogTable.kMainLog,"Auto/BrakeMode", true);
		
		// set control mode to VBus mode (% pwr) 
		mFrontLeft.changeControlMode(TalonSRX.TalonControlMode.PercentVbus);
		mFrontRight.changeControlMode(TalonSRX.TalonControlMode.PercentVbus);
		//mBackLeft.changeControlMode(TalonSRX.TalonControlMode.PercentVbus);
		//mBackRight.changeControlMode(TalonSRX.TalonControlMode.PercentVbus);

		// turn all motors to zero power		
		mFrontLeft.set(0);
		mFrontRight.set(0);
		
		//mBackLeft.set(0);
		//mBackRight.set(0);
		
	}

	private static void configureMotorsMagic() 
	{		
		// for auto - brake mode enabled
	    mFrontLeft.enableBrakeMode(false);
		mFrontRight.enableBrakeMode(false);
		
		//mBackLeft.enableBrakeMode(false);
		//mBackRight.enableBrakeMode(false);
		
		//ioComm.putBoolean(InputOutputComm.LogTable.kMainLog,"Auto/BrakeMode", false);
		
        // configure left and right front motors for magic motion (closed-loop position control)
		mFrontRight.changeControlMode(TalonSRX.TalonControlMode.MotionMagic);
        mFrontLeft.changeControlMode(TalonSRX.TalonControlMode.MotionMagic);	

        // Assign back right to follow front right
        //mBackRight.changeControlMode(CANTalon.TalonControlMode.Follower);
        //mBackRight.set(HardwareIDs.RIGHT_FRONT_TALON_ID);
        
        // Assign back left to follow front left
        //mBackLeft.changeControlMode(CANTalon.TalonControlMode.Follower);
        //mBackRight.set(HardwareIDs.LEFT_FRONT_TALON_ID);		
        
	}

	public static void resetPos()
	{		
		// reset front left and right encoder pulses to zero
		mFrontLeft.setPosition(0);
		mFrontRight.setPosition(0);
	}	 	        

	public static double getDistanceInches() {
		
		// query encoder for pulses so far - deprecated in 2018
		//double rightPos = mFrontRight.getPosition() * INCHES_PER_REV;
		//double leftPos = mFrontLeft.getPosition() * INCHES_PER_REV;
		
		// Encoders now read only raw encoder values - convert raw to inches directly
		double rightPos = mFrontRight.getPosition()*INCHES_PER_ENCODER_PULSE;
		double leftPos = mFrontLeft.getPosition()*INCHES_PER_ENCODER_PULSE;
				
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
		drive(leftSpeed, rightSpeed, 0.0);		 
	}
	
	public static void autoMagicStraight(double targetPosInches, int speedRpm, int accelRpm) {
		
		int nativeUnitsPer100ms = (int) ((double)speedRpm * RPM_TO_UNIT_PER_100MS);
		int accelNativeUnits = (int) ((double)accelRpm * RPM_TO_UNIT_PER_100MS);
		
        // left front drive straight - uses motion magic
		mFrontLeft.setMotionMagicCruiseVelocity(nativeUnitsPer100ms);
		mFrontLeft.setMotionMagicAcceleration(accelNativeUnits);
		mFrontLeft.set(targetPosInches/INCHES_PER_ENCODER_PULSE);
		
        // right front drive straight - uses motion magic
		mFrontRight.setMotionMagicCruiseVelocity(nativeUnitsPer100ms);
		mFrontRight.setMotionMagicAcceleration(accelNativeUnits);
		mFrontRight.set(targetPosInches/INCHES_PER_ENCODER_PULSE);
		
		// left and right back motors are following front motors
	}
	
	public static void autoMagicTurn(double targetPosInchesLeft, double targetPosInchesRight, int speedRpm) {

		int nativeUnitsPer100ms = (int) ((double)speedRpm * RPM_TO_UNIT_PER_100MS);
		
        // left front drive straight - uses motion magic
		mFrontLeft.setMotionMagicCruiseVelocity(nativeUnitsPer100ms);
		mFrontLeft.setMotionMagicAcceleration(nativeUnitsPer100ms);
		mFrontLeft.set(targetPosInchesLeft/INCHES_PER_ENCODER_PULSE);
		
        // right front drive straight - uses motion magic
		mFrontRight.setMotionMagicCruiseVelocity(nativeUnitsPer100ms);
		mFrontRight.setMotionMagicAcceleration(nativeUnitsPer100ms);
		mFrontRight.set(targetPosInchesRight/INCHES_PER_ENCODER_PULSE);
		
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
		drive(leftValue,rightValue,0.0);
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
	public static void drive(double leftValue, double rightValue, double strafe) {
		
		double rightMotorPolarity = -1.0;  // right motor is inverted 
		double leftMotorPolarity = 1.0;    // left motor is not inverted

		String leftSpeedStr = String.format("%.2f", leftValue);
		String rightSpeedStr = String.format("%.2f", rightValue);
		String myString2 = new String("leftSpeed = " + leftSpeedStr + " rightSpeed = " + rightSpeedStr);
		//System.out.println(myString2);
		//ioComm.putString(InputOutputComm.LogTable.kMainLog,"Auto/AutoDrive", myString2);

		// set motor values directly
		mFrontLeft.set(leftMotorPolarity*leftValue);
		//mBackLeft.set(leftMotorPolarity*leftValue);		
		mFrontRight.set(rightMotorPolarity*rightValue);
		//mBackRight.set(rightMotorPolarity*rightValue);
	}
	
	public static void driveDirection(double angle, double speed) {
		double gyroAngle = NavXSensor.getAngle();	
		double driveAngle = (angle-gyroAngle)*GYRO_CORRECT_COEFF;
		drive(driveAngle+speed, -driveAngle+speed, 0);
	}
	
	public static void turnToDirection(double angle, double power) {
		double gyroAngle = NavXSensor.getAngle();
		double driveAngle = (angle-gyroAngle)*(1/360)*power;
		drive(driveAngle, -driveAngle, 0);
	}
	
	public static void driveForward(double forwardVel) {
		drive(forwardVel, forwardVel, 0);
	}
	
	public static void rotate(double angularVel) {
		drive(angularVel, -angularVel, 0);
	}
	
	public static void driveVelocity(double forwardVel, double angularVel) {
		drive((forwardVel+angularVel)/2.0,(forwardVel-angularVel)/2.0,0);
	}
		
	//Turn methods
	//===================================================
	public static void rotateLeft(double speed) {		
		drive(-speed, speed, 0);
	}

	public static void rotateRight(double speed) {
		drive(speed, -speed, 0);
	}
}
