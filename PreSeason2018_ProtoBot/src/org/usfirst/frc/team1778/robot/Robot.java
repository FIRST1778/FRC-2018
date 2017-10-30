package org.usfirst.frc.team1778.robot;

import FreezyDrive.FreezyDriveTrain;
import NetworkComm.InputOutputComm;
import NetworkComm.RPIComm;
import StateMachine.AutoStateMachine;
import Systems.AutoDriveAssembly;
import Systems.BallManagement;
import Systems.CameraControl;
import Systems.ClimberAssembly;
import Systems.NavXSensor;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends IterativeRobot {
	final String defaultAuto = "Default";
	final String customAuto = "My Auto";
	String autoSelected;
	SendableChooser<String> chooser = new SendableChooser<>();

	protected AutoStateMachine autoSM;
	protected RPIComm rpiComm;
	protected InputOutputComm ioComm;
	protected AutoDriveAssembly autoDrive;
	protected FreezyDriveTrain freezyDrive;
	protected BallManagement ballCtrl;
	protected CameraControl camCtrl;
	protected ClimberAssembly climber;
	protected NavXSensor navX;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		chooser.addDefault("Default Auto", defaultAuto);
		chooser.addObject("My Auto", customAuto);
		SmartDashboard.putData("Auto choices", chooser);
		
		rpiComm = RPIComm.GetInstance();
		ioComm = InputOutputComm.GetInstance();
		autoDrive = AutoDriveAssembly.GetInstance();
		freezyDrive = FreezyDriveTrain.GetInstance();
		ballCtrl = BallManagement.GetInstance();
		camCtrl = CameraControl.GetInstance();
		climber = ClimberAssembly.GetInstance();
		
		autoSM = new AutoStateMachine();
		
    	ioComm.putString(InputOutputComm.LogTable.kMainLog,"MainLog","robot initialized...");        

	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit() {
		autoSelected = chooser.getSelected();
		// autoSelected = SmartDashboard.getString("Auto Selector",
		// defaultAuto);
		System.out.println("Auto selected: " + autoSelected);

    	ioComm.putString(InputOutputComm.LogTable.kMainLog,"MainLog","autonomous mode...");
    	autoDrive.autoInit(true, 0.0, false);
    	rpiComm.autoInit();
    	camCtrl.autoInit();
    	ballCtrl.autoInit();
    	
    	autoSM.start();

	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
    	rpiComm.updateValues();
    	
    	autoSM.process();
 
    	// debug only
    	autoDrive.getDistanceInches();
    	getGyroAngle();
   	
	}

	public void teleopInit() {
    	ioComm.putString(InputOutputComm.LogTable.kMainLog,"MainLog","teleop mode...");

    	rpiComm.teleopInit();
    	ballCtrl.teleopInit();  	
    	freezyDrive.teleopInit();	
    	camCtrl.teleopInit();
    	climber.teleopInit();
		
	}
	
	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
    	rpiComm.updateValues();  
        ballCtrl.teleopPeriodic();	
        freezyDrive.teleopPeriodic();   
        camCtrl.teleopPeriodic();
        climber.teleopPeriodic();
	}
	
    /**
     * This function is called periodically during operator control
     */
    
	private double getGyroAngle() {
		//double gyroAngle = 0.0;
		//double gyroAngle = navX.getYaw();  // -180 deg to +180 deg
		double gyroAngle = navX.getAngle();  // continuous angle (can be larger than 360 deg)
		
		//System.out.println("getGyroAngle:  Gyro angle = " + gyroAngle);
			
		// send output data for test & debug
	    ioComm.putBoolean(InputOutputComm.LogTable.kMainLog,"Auto/IMU_Connected",navX.isConnected());
	    ioComm.putBoolean(InputOutputComm.LogTable.kMainLog,"Auto/IMU_Calibrating",navX.isCalibrating());

		//System.out.println("gyroAngle = " + gyroAngle);
	    ioComm.putDouble(InputOutputComm.LogTable.kMainLog,"Auto/GyroAngle", gyroAngle);		

		return gyroAngle;
	}
	
    public void disabledInit() {
    	autoDrive.disabledInit();
    	ballCtrl.resetMotors();
    	rpiComm.disabledInit();
    }

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
	}
}

