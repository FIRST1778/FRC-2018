/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team1778.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends IterativeRobot {
	
	private final int TIMEOUT_MS = 0;  // set to zero if skipping confirmation
	private final int PIDLOOP_IDX = 0;  // set to zero if primary loop
	private final int PROFILE_SLOT = 0;

	private final int MASTER_TALON_ID = 4;
	private final int SLAVE_TALON_ID = 5;
	
	private final int SPARK_PWM_ID = 0;
	
	private TalonSRX master, slave;
	private Spark sparkMotor;
	
	// motion strength (%VBus - max is 1.0)
	private final double MOTION_IN_LEVEL = -0.25;
	private final double MOTION_OUT_LEVEL = 0.25;
	private final double DEAD_ZONE_THRESHOLD = 0.05;
	
	private Joystick gamepad;
	private final int GAMEPAD_ID = 1;
	private final int FOLLOWER_BUTTON = 1;
	private final int MIRROR_BUTTON = 2;
	private final int UNLINKED_BUTTON = 3;
	private final int POSITION_BUTTON = 4;
	
	private boolean master_open_loop = true;
	
	private final int TALON_AXIS = 2;
	private final int TALON_BUTTON = 5;
	
	private final int SPARK_AXIS = 3;
	private final int SPARK_BUTTON = 6;
	
	// PIDF values - proto.bot - initial 
	private final double kP = 7.0;
	private final double kI = 0.0;  // Integral not needed for closed loop position control
	private final double kD = 0.0;
	private final double kF = 0.0;  // Feedforward not used for closed loop position control
	
	private final int nativeUnitVel = 500;
	private final int nativeUnitAccel = 250;
	
	private final int targetPulses = 256;  // arbitrary encoder position to command - quarter turn
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		
		InputOutputComm.initialize();
		
		gamepad = new Joystick(GAMEPAD_ID);
		
		master = new TalonSRX(MASTER_TALON_ID);
		configureMotor(master, false, true, kP, kI, kD, kF);		
		resetPos();

		master.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
		master.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);

		slave = new TalonSRX(SLAVE_TALON_ID);
		slave.setInverted(true);
		
		sparkMotor = new Spark(SPARK_PWM_ID);
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * <p>You can add additional auto modes by adding additional comparisons to
	 * the switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit() {
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		checkMotorControls();
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
	
	private void checkMotorControls() {			

		if (gamepad.getRawButton(FOLLOWER_BUTTON)) {
			resetMotors();
			configureMotor(slave,false,master.getDeviceID());
			master_open_loop = true;
			InputOutputComm.putString(InputOutputComm.LogTable.kMainLog,"MotorTest/TestType", "Follower");

		}
		
		if (gamepad.getRawButton(MIRROR_BUTTON)) {
			resetMotors();
			configureMotor(slave,true,master.getDeviceID());
			master_open_loop = true;
			InputOutputComm.putString(InputOutputComm.LogTable.kMainLog,"MotorTest/TestType", "Mirror");
		}
		
		if (gamepad.getRawButton(UNLINKED_BUTTON)) {
			resetMotors();
			slave.setInverted(false);
			slave.set(ControlMode.PercentOutput, 0.0);
			master_open_loop = true;
			InputOutputComm.putString(InputOutputComm.LogTable.kMainLog,"MotorTest/TestType", "Unlinked");
		}
		
		if (gamepad.getRawButton(POSITION_BUTTON)) {
			configureMotor(master,false,true,kP,kI,kD,kF);
			configureMotor(slave,false,master.getDeviceID());
			goToPos(targetPulses);
			master_open_loop = false;
			InputOutputComm.putString(InputOutputComm.LogTable.kMainLog,"MotorTest/TestType", "Position");
		}
				
		// set master talon level
		if (master_open_loop) {
			double motorLevel = gamepad.getRawAxis(TALON_AXIS);
			if (Math.abs(motorLevel) > DEAD_ZONE_THRESHOLD)
				motorLevel = MOTION_IN_LEVEL;
			else if (gamepad.getRawButton(TALON_BUTTON))
				motorLevel = MOTION_OUT_LEVEL;
			else
				motorLevel = 0.0;	
		
			master.set(ControlMode.PercentOutput, motorLevel);
			InputOutputComm.putDouble(InputOutputComm.LogTable.kMainLog,"MotorTest/TalonLevel", motorLevel);
		}

		// set spark level
		double sparkLevel = gamepad.getRawAxis(SPARK_AXIS);
		if (Math.abs(sparkLevel) > DEAD_ZONE_THRESHOLD)
			sparkLevel = MOTION_IN_LEVEL;
		else if (gamepad.getRawButton(SPARK_BUTTON))
			sparkLevel = MOTION_OUT_LEVEL;
		else
			sparkLevel = 0.0;		
		sparkMotor.set(sparkLevel);
		InputOutputComm.putDouble(InputOutputComm.LogTable.kMainLog,"MotorTest/SparkLevel", sparkLevel);

	}
	
	private void resetMotors()
	{		
		// turn all motors to zero power (rear motors follow front motors)
		master.set(ControlMode.PercentOutput,0.0);
	}
	
	private void resetPos()
	{		
		// reset encoder pulses to zero
		master.setSelectedSensorPosition(0, PIDLOOP_IDX, TIMEOUT_MS);
	}	 	        
	
	
	private void goToPos(int targetPulses)
	{
		master.set(ControlMode.MotionMagic, targetPulses);
	}
	
    // closed-loop motor configuration
    private void configureMotor(TalonSRX _talon, boolean revMotor, boolean alignSensor,
    									double pCoeff, double iCoeff, double dCoeff, double fCoeff)
    {
    	_talon.setInverted(revMotor);
    	
    	_talon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, PIDLOOP_IDX, TIMEOUT_MS);
    	_talon.setSensorPhase(alignSensor); 
    	
    	_talon.selectProfileSlot(PROFILE_SLOT, PIDLOOP_IDX);
    	_talon.config_kP(PROFILE_SLOT, pCoeff, TIMEOUT_MS);
    	_talon.config_kI(PROFILE_SLOT, iCoeff, TIMEOUT_MS);
    	_talon.config_kD(PROFILE_SLOT, dCoeff, TIMEOUT_MS);
    	_talon.config_kF(PROFILE_SLOT, fCoeff, TIMEOUT_MS);
    	_talon.configMotionCruiseVelocity(nativeUnitVel, TIMEOUT_MS);
    	_talon.configMotionAcceleration(nativeUnitAccel, TIMEOUT_MS);
    	
    	_talon.setNeutralMode(NeutralMode.Brake);
 
    }
    
    // open-loop motor configuration (and possibly follower)
    private void configureMotor(TalonSRX _talon, boolean revMotor, int talonIDToFollow)
    {
    	_talon.setInverted(revMotor);
    	
    	if (talonIDToFollow > 0)
    		_talon.set(ControlMode.Follower, (double)talonIDToFollow);
    	
    	_talon.setNeutralMode(NeutralMode.Brake);
   	
    }

}
