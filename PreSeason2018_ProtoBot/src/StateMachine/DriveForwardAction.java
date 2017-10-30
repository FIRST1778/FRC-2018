package StateMachine;

import java.util.prefs.Preferences;

import NetworkComm.InputOutputComm;
import Systems.AutoDriveAssembly;
import Systems.NavXSensor;

public class DriveForwardAction extends Action {
	
	private String name;
	private double speed = 0.0;
	private boolean resetGyro = true;
	private double headingDeg = 0.0;
	private AutoDriveAssembly autoDrive;
	private InputOutputComm ioComm;
	private NavXSensor navX;
	
	public DriveForwardAction(double speed, boolean resetGyro, double headingDeg)
	{
		this.name = "<Drive Forward Action>";		
		this.speed = speed;
		this.resetGyro = resetGyro;
		this.headingDeg = headingDeg;   // absolute heading to use if not resetting gyro

		autoDrive = AutoDriveAssembly.GetInstance();
		navX = NavXSensor.GetInstance();
		ioComm = InputOutputComm.GetInstance();
	}
	
	public DriveForwardAction(String name, double speed, boolean resetGyro, double headingDeg)
	{
		this.name =  name;
		this.speed = speed;
		this.resetGyro = resetGyro;
		this.headingDeg = headingDeg;   // absolute heading to use if not resetting gyro
				
		autoDrive = AutoDriveAssembly.GetInstance();
		navX = NavXSensor.GetInstance();
		ioComm = InputOutputComm.GetInstance();
	}
	
	private double getGyroAngle() {
		//double gyroAngle = 0.0;
		//double gyroAngle = navX.getYaw();  // -180 deg to +180 deg
		double gyroAngle = navX.getAngle();  // continuous angle (can be larger than 360 deg)
		
		//System.out.println("autoPeriodicStraight:  Gyro angle = " + gyroAngle);
			
		// send output data for test & debug
	    ioComm.putBoolean(InputOutputComm.LogTable.kMainLog,"Auto/IMU_Connected",navX.isConnected());
	    ioComm.putBoolean(InputOutputComm.LogTable.kMainLog,"Auto/IMU_Calibrating",navX.isCalibrating());

		//System.out.println("gyroAngle = " + gyroAngle);
	    ioComm.putDouble(InputOutputComm.LogTable.kMainLog,"Auto/GyroAngle", gyroAngle);		

		return gyroAngle;
	}
	
	// action entry
	public void initialize() {
		// do some drivey initialization
		
		autoDrive.autoInit(resetGyro, headingDeg, false);
		
		super.initialize();
	}
	
	// called periodically
	public void process()  {
		
		// do some drivey stuff
				
		autoDrive.autoGyroStraight(speed);

		// get gyro angle 
		// (not used for anything else here except reporting to driver)
		getGyroAngle();
		
		super.process();
	}
	
	// state cleanup and exit
	public void cleanup() {
		// do some drivey cleanup
					
		autoDrive.autoStop();
		
		// cleanup base class
		super.cleanup();
	}
	
	public void persistWrite(int counter, Preferences prefs) {

		// create node for action
		Preferences actionPrefs = prefs.node(counter + "_" + this.name);
	
		// store action details
		actionPrefs.put("class",this.getClass().toString());
		actionPrefs.putDouble("speed",this.speed);
	}

}
