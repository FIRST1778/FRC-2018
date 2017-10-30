package Systems;

import com.ctre.phoenix.MotorControl.CAN.TalonSRX;
import com.ctre.phoenix.MotorControl.SmartMotorController.FeedbackDevice;

import NetworkComm.InputOutputComm;
import Utility.HardwareIDs;

//Chill Out 1778 class for controlling the drivetrain during auto

public class AutoDriveAssembly {
	
    // singleton class elements (ensures only one instance of this class)
	private static final AutoDriveAssembly instance = new AutoDriveAssembly();
	   
	private AutoDriveAssembly() {
		
		ioComm = InputOutputComm.GetInstance();
		navX = NavXSensor.GetInstance();
		turnCtrl = TurnController.GetInstance();
		
		// instantiate motion profile motor control objects
        mFrontLeft = new TalonSRX(HardwareIDs.LEFT_FRONT_TALON_ID);
        mFrontLeft.reverseOutput(LEFT_REVERSE_MOTOR);	   // left motor PID polarity (magic motion mode only)
		mBackLeft = new TalonSRX(HardwareIDs.LEFT_REAR_TALON_ID);
		mBackLeft.reverseOutput(false);	   // left back motor feedback polarity (follower mode only)

		mFrontRight = new TalonSRX(HardwareIDs.RIGHT_FRONT_TALON_ID);
		mFrontRight.reverseOutput(RIGHT_REVERSE_MOTOR);  // right motor PID polarity (magic motion mode only)
		mFrontRight = new TalonSRX(HardwareIDs.RIGHT_REAR_TALON_ID);
		mFrontRight.reverseOutput(true);  // right back motor feedback polarity (follower mode only)
        
        // configure left front motor encoder and PID
        mFrontLeft.setFeedbackDevice(FeedbackDevice.QuadEncoder);
        mFrontLeft.configEncoderCodesPerRev(ENCODER_PULSES_PER_REV);
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
        mFrontRight.configEncoderCodesPerRev(ENCODER_PULSES_PER_REV);
        mFrontRight.reverseSensor(RIGHT_REVERSE_SENSOR);   // right motor encoder polarity
        mFrontRight.setProfile(0);
        mFrontRight.setP(P_COEFF);
        mFrontRight.setI(I_COEFF);
        mFrontRight.setD(D_COEFF);
        mFrontRight.setF(F_COEFF);
        mFrontRight.setMotionMagicCruiseVelocity(0);
        mFrontRight.setMotionMagicAcceleration(0);
	}
		
	public static AutoDriveAssembly GetInstance() {
		return instance;
	}
		
	// instance data and methods
	private InputOutputComm ioComm;
	private NavXSensor navX;
	private TurnController turnCtrl;
	
	private static final double AUTO_DRIVE_ANGLE_CORRECT_COEFF = 0.02;
	private static final double GYRO_CORRECT_COEFF = 0.03;
			
	// smart controllers (motion profiling)
	private TalonSRX mFrontLeft, mFrontRight;
	private TalonSRX mBackLeft, mBackRight;
		
	// used as angle baseline (if we don't reset gyro)
	private double initialAngle = 0.0;
	
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
	
	public static final int ENCODER_PULSES_PER_REV = 256;  // 63R  - on the competition bot front motors
	//public static final double INCHES_PER_REV = (6 * 3.14159);   // 6-in diameter wheel (theoretical)
	public static final double INCHES_PER_REV = (5.9 * 3.14159);   // 5.9-in diameter wheel (worn)
			
	// PIDF values - comp.bot version tuned 7/20/2017
	private static final double P_COEFF = 20.0;
	private static final double I_COEFF = 0.0;  // Integral not needed for closed loop position control
	private static final double D_COEFF = 16.0;
	private static final double F_COEFF = 0.0;  // Feedforward not used for closed loop position control
		
	private void resetMotors()
	{
		// disable brake mode (all motors on coast)
	    mFrontLeft.enableBrakeMode(false);
		mFrontRight.enableBrakeMode(false);
		mBackLeft.enableBrakeMode(false);
		mBackRight.enableBrakeMode(false);
		ioComm.putBoolean(InputOutputComm.LogTable.kMainLog,"Auto/BrakeMode", false);
		
		// reset control mode to VBus mode (% pwr) 
		mFrontLeft.changeControlMode(TalonSRX.TalonControlMode.PercentVbus);
		mFrontRight.changeControlMode(TalonSRX.TalonControlMode.PercentVbus);
		mBackLeft.changeControlMode(TalonSRX.TalonControlMode.PercentVbus);
		mBackRight.changeControlMode(TalonSRX.TalonControlMode.PercentVbus);

		// turn all motors to zero power
		mFrontLeft.set(0);
		mFrontRight.set(0);
		mBackLeft.set(0);
		mBackRight.set(0);
		
	}
		
	private void configureMotorsVbus() 
	{		
		// for auto - brake mode enabled
	    mFrontLeft.enableBrakeMode(true);
		mFrontRight.enableBrakeMode(true);
		mBackLeft.enableBrakeMode(true);
		mBackRight.enableBrakeMode(true);
		ioComm.putBoolean(InputOutputComm.LogTable.kMainLog,"Auto/BrakeMode", true);
		
		// set control mode to VBus mode (% pwr) 
		mFrontLeft.changeControlMode(TalonSRX.TalonControlMode.PercentVbus);
		mFrontRight.changeControlMode(TalonSRX.TalonControlMode.PercentVbus);
		mBackLeft.changeControlMode(TalonSRX.TalonControlMode.PercentVbus);
		mBackRight.changeControlMode(TalonSRX.TalonControlMode.PercentVbus);

		// turn all motors to zero power		
		mFrontLeft.set(0);
		mFrontRight.set(0);
		mBackLeft.set(0);
		mBackRight.set(0);
		
	}

	private void configureMotorsMagic() 
	{		
		// for auto - brake mode enabled
	    mFrontLeft.enableBrakeMode(false);
		mFrontRight.enableBrakeMode(false);
		mBackLeft.enableBrakeMode(false);
		mBackRight.enableBrakeMode(false);
		ioComm.putBoolean(InputOutputComm.LogTable.kMainLog,"Auto/BrakeMode", false);
		
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

	public void resetPos()
	{		
		// reset front left and right encoder pulses to zero
		mFrontLeft.setPosition(0);
		mFrontRight.setPosition(0);
	}	 	        

	public double getDistanceInches() {
		
		// query encoder for pulses so far
		double rightPos = mFrontRight.getPosition() * INCHES_PER_REV;
		double leftPos = mFrontLeft.getPosition() * INCHES_PER_REV;
				
		String posStr = String.format("%.2f", rightPos);
		ioComm.putString(InputOutputComm.LogTable.kMainLog,"Auto/EncoderRight", posStr);
		posStr = String.format("%.2f", leftPos);
		ioComm.putString(InputOutputComm.LogTable.kMainLog,"Auto/EncoderLeft", posStr);
		
		return rightPos;
	}
	
	public void autoInit(boolean resetGyro, double headingDeg, boolean magicMotion) {
				
		if (resetGyro) {
			navX.reset();
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
					
	public void autoGyroStraight(double speed) {
		// autonomous operation of drive straight - uses gyro
		
		double gyroAngle = navX.getAngle();
		
		// subtract the initial angle offset, if any
		gyroAngle -= initialAngle;
		
		// calculate speed adjustment for left and right sides (negative sign added as feedback)
		double driveAngle = -gyroAngle * AUTO_DRIVE_ANGLE_CORRECT_COEFF;
				
		double leftSpeed = speed+driveAngle;		
		double rightSpeed = speed-driveAngle;
				
		// adjust speed of left and right sides
		drive(leftSpeed, rightSpeed, 0.0);		 
	}
	
	public void autoMagicStraight(double targetPosRevs, int speedRpm) {
		
        // left front drive straight - uses motion magic
		mFrontLeft.setMotionMagicCruiseVelocity(speedRpm);
		mFrontLeft.setMotionMagicAcceleration(speedRpm);
		mFrontLeft.set(targetPosRevs);
		
        // right front drive straight - uses motion magic
		mFrontRight.setMotionMagicCruiseVelocity(speedRpm);
		mFrontRight.setMotionMagicAcceleration(speedRpm);
		mFrontRight.set(targetPosRevs);	
		
		// left and right back motors are following front motors
	}
	
	public void autoMagicTurn(double targetPosRevsLeft, double targetPosRevsRight, int speedRpm) {
		
        // left front drive straight - uses motion magic
		mFrontLeft.setMotionMagicCruiseVelocity(speedRpm);
		mFrontLeft.setMotionMagicAcceleration(speedRpm);
		mFrontLeft.set(targetPosRevsLeft);
		
        // right front drive straight - uses motion magic
		mFrontRight.setMotionMagicCruiseVelocity(speedRpm);
		mFrontRight.setMotionMagicAcceleration(speedRpm);
		mFrontRight.set(targetPosRevsRight);	
		
		// left and right back motors are following front motors
	}
	
	public void autoStop() {
		resetMotors();
	}
	
	// auto PID turn methods
	public void autoPidTurnStart(double angleDeg, double speed) {
		resetMotors();
		turnCtrl.setAngle(angleDeg, speed);
		turnCtrl.enable();
	}
	
	public void autoPidTurnProcess() {
		double leftValue = turnCtrl.getLeft();
		double rightValue = turnCtrl.getRight();
		drive(leftValue,rightValue,0.0);
	}
	
	public void autoPidTurnStop() {
		turnCtrl.disable();
		resetMotors();
	}
		
	public void teleopInit() {
	}
	
	public void teleopPeriodic() {	
	}
	
	public void disabledInit( )  {
		resetMotors();
	}
	
	// CORE DRIVE METHOD
	// Assumes parameters are PercentVbus (0.0 to 1.0)
	public void drive(double leftValue, double rightValue, double strafe) {
		
		double rightMotorPolarity = -1.0;  // right motor is inverted 
		double leftMotorPolarity = 1.0;    // left motor is not inverted

		String leftSpeedStr = String.format("%.2f", leftValue);
		String rightSpeedStr = String.format("%.2f", rightValue);
		String myString2 = new String("leftSpeed = " + leftSpeedStr + " rightSpeed = " + rightSpeedStr);
		//System.out.println(myString2);
		ioComm.putString(InputOutputComm.LogTable.kMainLog,"Auto/AutoDrive", myString2);

		// set motor values directly
		mFrontLeft.set(leftMotorPolarity*leftValue);
		mBackLeft.set(leftMotorPolarity*leftValue);		
		mFrontRight.set(rightMotorPolarity*rightValue);
		mBackRight.set(rightMotorPolarity*rightValue);
	}
	
	public void driveDirection(double angle, double speed) {
		double gyroAngle = navX.getAngle();	
		double driveAngle = (angle-gyroAngle)*GYRO_CORRECT_COEFF;
		drive(driveAngle+speed, -driveAngle+speed, 0);
	}
	
	public void turnToDirection(double angle, double power) {
		double gyroAngle = navX.getAngle();
		double driveAngle = (angle-gyroAngle)*(1/360)*power;
		drive(driveAngle, -driveAngle, 0);
	}
	
	public void driveForward(double forwardVel) {
		drive(forwardVel, forwardVel, 0);
	}
	
	public void rotate(double angularVel) {
		drive(angularVel, -angularVel, 0);
	}
	
	public void driveVelocity(double forwardVel, double angularVel) {
		drive((forwardVel+angularVel)/2.0,(forwardVel-angularVel)/2.0,0);
	}
		
	//Turn methods
	//===================================================
	public void rotateLeft(double speed) {		
		drive(-speed, speed, 0);
	}

	public void rotateRight(double speed) {
		drive(speed, -speed, 0);
	}
}
