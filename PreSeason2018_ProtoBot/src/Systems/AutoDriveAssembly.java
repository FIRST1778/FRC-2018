package Systems;

import com.ctre.phoenix.MotorControl.CAN.TalonSRX;
import com.ctre.phoenix.MotorControl.ControlMode;
import com.ctre.phoenix.MotorControl.FeedbackDevice;
import com.ctre.phoenix.MotorControl.NeutralMode;

import NetworkComm.InputOutputComm;
import Utility.HardwareIDs;

//Chill Out 1778 class for controlling the drivetrain during auto

public class AutoDriveAssembly {
		
	private static boolean initialized = false;
	private static final int TIMEOUT_MS = 100;
	private static final int PROFILE_SLOT = 0;
	
	public static void initialize() {
		
		if (initialized)
			return;
		
		//ioComm = InputOutputComm.GetInstance();
		NavXSensor.initialize();
		TurnController.initialize();
		
		// instantiate motion profile motor control objects
        mFrontLeft = new TalonSRX(HardwareIDs.LEFT_FRONT_TALON_ID);
        //mFrontLeft.reverseOutput(LEFT_REVERSE_MOTOR);	       // deprecated 2018
        mFrontLeft.setInverted(LEFT_REVERSE_MOTOR);  // left motor PID polarity (magic motion mode only)
        
        //mBackLeft = new TalonSRX(HardwareIDs.LEFT_REAR_TALON_ID);
		//mBackLeft.reverseOutput(false);	   // left back motor feedback polarity (follower mode only)

		mFrontRight = new TalonSRX(HardwareIDs.RIGHT_FRONT_TALON_ID);
		//mFrontRight.reverseOutput(RIGHT_REVERSE_MOTOR);      // deprecated 2018
		mFrontRight.setInverted(RIGHT_REVERSE_MOTOR);  // right motor PID polarity (magic motion mode only)

		//mBackRight = new TalonSRX(HardwareIDs.RIGHT_REAR_TALON_ID);
		//mBackRight.reverseOutput(true);  // right back motor feedback polarity (follower mode only)
        
        // configure left front motor encoder and PID
        //mFrontLeft.setFeedbackDevice(FeedbackDevice.QuadEncoder); // deprecated 2018
        mFrontLeft.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, TIMEOUT_MS);
        
        //mFrontLeft.configEncoderCodesPerRev(ENCODER_PULSES_PER_REV);    // deprecated 2018
        //mFrontLeft.reverseSensor(LEFT_REVERSE_SENSOR);       // deprecated 2018
        mFrontLeft.setSensorPhase(ALIGNED_LEFT_SENSOR);   // left motor encoder polarity
        
        /* deprecated 2018
        mFrontLeft.setProfile(0);      
        mFrontLeft.setP(P_COEFF);
        mFrontLeft.setI(I_COEFF);
        mFrontLeft.setD(D_COEFF);
        mFrontLeft.setF(F_COEFF);
        mFrontLeft.setMotionMagicCruiseVelocity(0);
        mFrontLeft.setMotionMagicAcceleration(0);
        */
        mFrontLeft.config_kP(PROFILE_SLOT, P_COEFF, TIMEOUT_MS);
        mFrontLeft.config_kI(PROFILE_SLOT, I_COEFF, TIMEOUT_MS);
        mFrontLeft.config_kD(PROFILE_SLOT, D_COEFF, TIMEOUT_MS);
        mFrontLeft.config_kF(PROFILE_SLOT, F_COEFF, TIMEOUT_MS);
        mFrontLeft.selectProfileSlot(PROFILE_SLOT);
        mFrontLeft.configMotionCruiseVelocity(0, TIMEOUT_MS);
        mFrontLeft.configMotionAcceleration(0, TIMEOUT_MS);
        
        // configure right front motor encoder and PID
        //mFrontRight.setFeedbackDevice(FeedbackDevice.QuadEncoder);  // deprecated 2018
        mFrontRight.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 100);
        
       //mFrontRight.configEncoderCodesPerRev(ENCODER_PULSES_PER_REV);    // deprecated 2018
        //mFrontRight.reverseSensor(RIGHT_REVERSE_SENSOR);   // deprecated 2018
        mFrontRight.setSensorPhase(ALIGNED_RIGHT_SENSOR);   // right motor encoder polarity
        
        /* deprecated 2018
        mFrontRight.setProfile(0);
        mFrontRight.setP(P_COEFF);
        mFrontRight.setI(I_COEFF);
        mFrontRight.setD(D_COEFF);
        mFrontRight.setF(F_COEFF);
        mFrontRight.setMotionMagicCruiseVelocity(0);
        mFrontRight.setMotionMagicAcceleration(0);
        */
        mFrontRight.config_kP(PROFILE_SLOT, P_COEFF, TIMEOUT_MS);
        mFrontRight.config_kI(PROFILE_SLOT, I_COEFF, TIMEOUT_MS);
        mFrontRight.config_kD(PROFILE_SLOT, D_COEFF, TIMEOUT_MS);
        mFrontRight.config_kF(PROFILE_SLOT, F_COEFF, TIMEOUT_MS);
        mFrontRight.selectProfileSlot(PROFILE_SLOT);
        mFrontRight.configMotionCruiseVelocity(0, TIMEOUT_MS);
        mFrontRight.configMotionAcceleration(0, TIMEOUT_MS);
        
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
	
	//public static final boolean RIGHT_REVERSE_SENSOR = false;	// encoder polarity - right
	//public static final boolean LEFT_REVERSE_SENSOR = true;	// encoder polarity - left
	public static final boolean ALIGNED_RIGHT_SENSOR = true;	// encoder polarity - right
	public static final boolean ALIGNED_LEFT_SENSOR = false;    // encoder polarity - left
	
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
		mFrontLeft.setNeutralMode(NeutralMode.Coast);
		mFrontRight.setNeutralMode(NeutralMode.Coast);
		//mBackLeft.setNeutralMode(NeutralMode.Coast);
		//mBackRight.setNeutralMode(NeutralMode.Coast);

		//mFrontLeft.enableBrakeMode(false);    // deprecated 2018
		//mFrontRight.enableBrakeMode(false);	// deprecated 2018
		//mBackLeft.enableBrakeMode(false);		// deprecated 2018
		//mBackRight.enableBrakeMode(false);    // deprecated 2018
		
		//ioComm.putBoolean(InputOutputComm.LogTable.kMainLog,"Auto/BrakeMode", false);
		
		// reset control mode to VBus mode (% pwr) 
		mFrontLeft.enableVoltageCompensation(true);   // NOT tested
		mFrontRight.enableVoltageCompensation(true);  // NOT tested
		//mBackLeft.enableVoltageCompensation(true);   // NOT tested
		//mBackRight.enableVoltageCompensation(true);  // NOT tested

		//mFrontLeft.changeControlMode(TalonSRX.TalonControlMode.PercentVbus);    // deprecated 2018
		//mFrontRight.changeControlMode(TalonSRX.TalonControlMode.PercentVbus);   // deprecated 2018	
		//mBackLeft.changeControlMode(TalonSRX.TalonControlMode.PercentVbus);     // deprecated 2018
		//mBackRight.changeControlMode(TalonSRX.TalonControlMode.PercentVbus);    // deprecated 2018

		// turn all motors to zero power
		mFrontLeft.set(ControlMode.PercentOutput,0.0);
		mFrontRight.set(ControlMode.PercentOutput,0.0);		
		//mBackLeft.set(ControlMode.PercentOutput,0.0);
		//mBackRight.set(ControlMode.PercentOutput,0.0);

		//mFrontLeft.set(0);    // deprecated 2018
		//mFrontRight.set(0);   // deprecated 2018
		//mBackLeft.set(0);     // deprecated 2018
		//mBackRight.set(0);    // deprecated 2018
		
	}
		
	private static void configureMotorsVbus() 
	{		
		// for auto - brake mode enabled
		mFrontLeft.setNeutralMode(NeutralMode.Brake);
		mFrontRight.setNeutralMode(NeutralMode.Brake);
		//mBackLeft.setNeutralMode(NeutralMode.Brake);
		//mBackRight.setNeutralMode(NeutralMode.Brake);
		
		//ioComm.putBoolean(InputOutputComm.LogTable.kMainLog,"Auto/BrakeMode", true);
		
		// set control mode to VBus mode (% pwr) 
		//mFrontLeft.changeControlMode(TalonSRX.TalonControlMode.PercentVbus);   // deprecated 2018
		//mFrontRight.changeControlMode(TalonSRX.TalonControlMode.PercentVbus);  // deprecated 2018
		//mBackLeft.changeControlMode(TalonSRX.TalonControlMode.PercentVbus);    // deprecated 2018
		//mBackRight.changeControlMode(TalonSRX.TalonControlMode.PercentVbus);   // deprecated 2018

		// turn all motors to zero power		
		mFrontLeft.set(ControlMode.PercentOutput,0);
		mFrontRight.set(ControlMode.PercentOutput,0);		
		//mBackLeft.set(ControlMode.PercentOutput,0);
		//mBackRight.set(ControlMode.PercentOutput,0);
	
		//mFrontLeft.set(0);   // deprecated 2018
		//mFrontRight.set(0);  // deprecated 2018
		//mBackLeft.set(0);    // deprecated 2018
		//mBackRight.set(0);   // deprecated 2018
		
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
		
		//mFrontRight.changeControlMode(TalonSRX.TalonControlMode.MotionMagic);  // deprecated 2018
        //mFrontLeft.changeControlMode(TalonSRX.TalonControlMode.MotionMagic);   // deprecated 2018	

        // Assign back right to follow front right
        //mBackRight.changeControlMode(CANTalon.TalonControlMode.Follower);     // deprecated 2018
        //mBackRight.set(HardwareIDs.RIGHT_FRONT_TALON_ID);     				// deprecated 2018
        
        // Assign back left to follow front left
        //mBackLeft.changeControlMode(CANTalon.TalonControlMode.Follower);     // deprecated 2018
        //mBackRight.set(HardwareIDs.LEFT_FRONT_TALON_ID);		     			// deprecated 2018        
	}

	public static void resetPos()
	{		
		// reset front left and right encoder pulses to zero
		mFrontLeft.set(ControlMode.Position, 0);
		mFrontRight.set(ControlMode.Position, 0);	
		//mFrontLeft.setPosition(0);     		// deprecated 2018
		//mFrontRight.setPosition(0);			// deprecated 2018
	}	 	        

	public static double getDistanceInches() {
		
		// query encoder for pulses so far - deprecated in 2018
		//double rightPos = mFrontRight.getPosition() * INCHES_PER_REV;
		//double leftPos = mFrontLeft.getPosition() * INCHES_PER_REV;
		
		// Encoders now read only raw encoder values - convert raw to inches directly
		double rightPos = mFrontRight.getSelectedSensorPosition()*INCHES_PER_ENCODER_PULSE;
		double leftPos = mFrontLeft.getSelectedSensorPosition()*INCHES_PER_ENCODER_PULSE;
		//double rightPos = mFrontRight.getPosition()*INCHES_PER_ENCODER_PULSE;   // deprecated 2018
		//double leftPos = mFrontLeft.getPosition()*INCHES_PER_ENCODER_PULSE;		// deprecated 2018
				
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
		mFrontLeft.configMotionCruiseVelocity(nativeUnitsPer100ms, TIMEOUT_MS);
		mFrontLeft.configMotionAcceleration(accelNativeUnits, TIMEOUT_MS);
		mFrontLeft.set(ControlMode.Position, targetPosInches/INCHES_PER_ENCODER_PULSE);
		
        // right front drive straight - uses motion magic
        mFrontRight.configMotionCruiseVelocity(nativeUnitsPer100ms, TIMEOUT_MS);
        mFrontRight.configMotionAcceleration(accelNativeUnits, TIMEOUT_MS);
        mFrontRight.set(ControlMode.Position, targetPosInches/INCHES_PER_ENCODER_PULSE);

		/* deprecated 2018
		mFrontLeft.setMotionMagicCruiseVelocity(nativeUnitsPer100ms);
		mFrontLeft.setMotionMagicAcceleration(accelNativeUnits);
		mFrontLeft.set(targetPosInches/INCHES_PER_ENCODER_PULSE);

		mFrontRight.setMotionMagicCruiseVelocity(nativeUnitsPer100ms);
		mFrontRight.setMotionMagicAcceleration(accelNativeUnits);
		mFrontRight.set(targetPosInches/INCHES_PER_ENCODER_PULSE);
		*/
		
		// left and right back motors are following front motors
	}
	
	public static void autoMagicTurn(double targetPosInchesLeft, double targetPosInchesRight, int speedRpm, int accelRpm) {

		int nativeUnitsPer100ms = (int) ((double)speedRpm * RPM_TO_UNIT_PER_100MS);
		int accelNativeUnits = (int) ((double)accelRpm * RPM_TO_UNIT_PER_100MS);
		
        // left front drive straight - uses motion magic
		mFrontLeft.configMotionCruiseVelocity(nativeUnitsPer100ms, TIMEOUT_MS);
		mFrontLeft.configMotionAcceleration(accelNativeUnits, TIMEOUT_MS);
		mFrontLeft.set(ControlMode.Position, targetPosInchesLeft/INCHES_PER_ENCODER_PULSE);
		
        // right front drive straight - uses motion magic
        mFrontRight.configMotionCruiseVelocity(nativeUnitsPer100ms, TIMEOUT_MS);
        mFrontRight.configMotionAcceleration(accelNativeUnits, TIMEOUT_MS);
        mFrontRight.set(ControlMode.Position, targetPosInchesRight/INCHES_PER_ENCODER_PULSE);
		
		/* deprecated 2018
		mFrontLeft.setMotionMagicCruiseVelocity(nativeUnitsPer100ms);
		mFrontLeft.setMotionMagicAcceleration(nativeUnitsPer100ms);
		mFrontLeft.set(targetPosInchesLeft/INCHES_PER_ENCODER_PULSE);

		mFrontRight.setMotionMagicCruiseVelocity(nativeUnitsPer100ms);
		mFrontRight.setMotionMagicAcceleration(nativeUnitsPer100ms);
		mFrontRight.set(targetPosInchesRight/INCHES_PER_ENCODER_PULSE);
		*/
		
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
		mFrontLeft.set(ControlMode.PercentOutput, leftMotorPolarity*leftValue);
		mFrontRight.set(ControlMode.PercentOutput, rightMotorPolarity*rightValue);
		//mBackLeft.set(ControlMode.PercentOutput, leftMotorPolarity*rightValue);
		//mBackRight.set(ControlMode.PercentOutput, rightMotorPolarity*rightValue);
		
		//mFrontLeft.set(leftMotorPolarity*leftValue);           // deprecated 2018
		//mFrontRight.set(rightMotorPolarity*rightValue);        // deprecated 2018
		//mBackLeft.set(leftMotorPolarity*leftValue);            // deprecated 2018		
		//mBackRight.set(rightMotorPolarity*rightValue);         // deprecated 2018
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
