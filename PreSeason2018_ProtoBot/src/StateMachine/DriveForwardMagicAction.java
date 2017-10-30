package StateMachine;

import java.util.prefs.Preferences;

import NetworkComm.InputOutputComm;
import Systems.AutoDriveAssembly;

public class DriveForwardMagicAction extends Action {
	
	private String name;
	private double targetPosRevs = 0.0;
	private int speedRpm = 0;
	private boolean resetGyro = false;
	private double headingDeg = 0.0;   // angle to use if gyro not reset
	private AutoDriveAssembly autoDrive;
		
	public DriveForwardMagicAction(double targetPosInches, int speedRpm, boolean resetGyro, double headingDeg)
	{
		this.name = "<Drive Forward Magic Action>";		
		this.targetPosRevs = targetPosInches/AutoDriveAssembly.INCHES_PER_REV;
		this.speedRpm = speedRpm;
		this.resetGyro = resetGyro;
		this.headingDeg = headingDeg;

		autoDrive = AutoDriveAssembly.GetInstance();
	}
	
	public DriveForwardMagicAction(String name, double targetPosInches, int speedRpm, boolean resetGyro, double headingDeg)
	{
		this.name =  name;
		this.targetPosRevs = targetPosInches/AutoDriveAssembly.INCHES_PER_REV;
		this.speedRpm = speedRpm;
		this.resetGyro = resetGyro;
		this.headingDeg = headingDeg;
				
		autoDrive = AutoDriveAssembly.GetInstance();
	}
		
	// action entry
	public void initialize() {
		// do some drivey initialization
		
		autoDrive.autoInit(resetGyro, headingDeg, true);
		autoDrive.autoMagicStraight(targetPosRevs, speedRpm);
		
		super.initialize();
	}
	
	// called periodically
	public void process()  {
		
		// do some drivey stuff
						
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
		actionPrefs.putDouble("speedRpm",this.speedRpm);
	}

}
