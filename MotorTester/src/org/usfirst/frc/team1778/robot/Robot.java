/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team1778.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
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

	private final int MASTER_TALON_ID = 4;
	private final int SLAVE_TALON_ID = 5;
	
	private final int SPARK_PWM_ID = 1;
	
	private TalonSRX master, slave;
	private Spark sparkMotor;
	
	// motion strength (%VBus - max is 1.0)
	private final double MOTION_IN_LEVEL = -0.75;
	private final double MOTION_OUT_LEVEL = 0.75;
	private final double DEAD_ZONE_THRESHOLD = 0.05;
	
	private Joystick gamepad;
	private final int GAMEPAD_ID = 1;
	private final int FOLLOWER_BUTTON = 1;
	private final int MIRROR_BUTTON = 2;
	private final int UNLINKED_BUTTON = 3;
	
	private final int TALON_AXIS = 2;
	private final int TALON_BUTTON = 5;
	
	private final int SPARK_AXIS = 3;
	private final int SPARK_BUTTON = 6;
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		gamepad = new Joystick(GAMEPAD_ID);
		
		master = new TalonSRX(MASTER_TALON_ID);
		master.setInverted(false);
		
		//master.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
		//master.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);

		slave = new TalonSRX(SLAVE_TALON_ID);
		slave.setInverted(false);
		
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
			slave.setInverted(false);
			slave.set(ControlMode.Follower, master.getDeviceID());
		}
		
		if (gamepad.getRawButton(MIRROR_BUTTON)) {
			slave.setInverted(true);
			slave.set(ControlMode.Follower, master.getDeviceID());
		}
		
		if (gamepad.getRawButton(UNLINKED_BUTTON)) {
			slave.setInverted(false);
			slave.set(ControlMode.PercentOutput, 0.0);
		}
		
		// set master talon level
		double motorLevel = gamepad.getRawAxis(TALON_AXIS);
		if (Math.abs(motorLevel) > DEAD_ZONE_THRESHOLD)
			motorLevel = MOTION_IN_LEVEL;
		else if (gamepad.getRawButton(TALON_BUTTON))
			motorLevel = MOTION_OUT_LEVEL;
		else
			motorLevel = 0.0;		
		master.set(ControlMode.PercentOutput, motorLevel);

		// set spark level
		motorLevel = gamepad.getRawAxis(SPARK_AXIS);
		if (Math.abs(motorLevel) > DEAD_ZONE_THRESHOLD)
			motorLevel = MOTION_IN_LEVEL;
		else if (gamepad.getRawButton(SPARK_BUTTON))
			motorLevel = MOTION_OUT_LEVEL;
		else
			motorLevel = 0.0;		
		sparkMotor.set(motorLevel);

	}

}
