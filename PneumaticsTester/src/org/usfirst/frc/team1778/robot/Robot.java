/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team1778.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends IterativeRobot {

	private Joystick gamepad;
	private final int GAMEPAD_ID = 1;
	private final int PISTON1_PNEUMATIC_BUTTON_1 = 1;
	private final int PISTON1_PNEUMATIC_BUTTON_2 = 2;
	
	private final int PISTON2_PNEUMATIC_BUTTON_1 = 3;
	private final int PISTON2_PNEUMATIC_BUTTON_2 = 4;
	
	private final int PCM_ID = 2;	
	
	private final int PISTON1_FORWARD_CHANNEL = 0;
	private final int PISTON1_REVERSE_CHANNEL = 1;

	private final int PISTON2_FORWARD_CHANNEL = 2;
	private final int PISTON2_REVERSE_CHANNEL = 3;
	
	private Compressor compress;
	private DoubleSolenoid dSol1, dSol2;
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		gamepad = new Joystick(GAMEPAD_ID);
		compress = new Compressor(PCM_ID);	
		
		dSol1 = new DoubleSolenoid(PCM_ID, PISTON1_FORWARD_CHANNEL, PISTON1_REVERSE_CHANNEL);
		dSol1.set(DoubleSolenoid.Value.kOff);	
		
		dSol2 = new DoubleSolenoid(PCM_ID, PISTON2_FORWARD_CHANNEL, PISTON2_REVERSE_CHANNEL);
		dSol2.set(DoubleSolenoid.Value.kOff);	
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
		
		// if gamepad button 1 pressed, piston 1 go forward
		if (gamepad.getRawButton(PISTON1_PNEUMATIC_BUTTON_1))
		{
			dSol1.set(DoubleSolenoid.Value.kForward);
		}
		
		// if gamepad button 2 pressed, piston1 go reverse 
		if (gamepad.getRawButton(PISTON1_PNEUMATIC_BUTTON_2))
		{
			dSol1.set(DoubleSolenoid.Value.kReverse);
		}
		
		// if gamepad button 1 pressed, piston 2 go forward
		if (gamepad.getRawButton(PISTON2_PNEUMATIC_BUTTON_1))
		{
			dSol2.set(DoubleSolenoid.Value.kForward);
		}
		
		// if gamepad button 2 pressed, piston2 go reverse 
		if (gamepad.getRawButton(PISTON2_PNEUMATIC_BUTTON_2))
		{
			dSol2.set(DoubleSolenoid.Value.kReverse);
		}
		
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
}
