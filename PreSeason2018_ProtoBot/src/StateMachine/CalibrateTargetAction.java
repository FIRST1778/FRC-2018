package StateMachine;

import java.util.prefs.Preferences;

import Systems.AutoDriveAssembly;
import NetworkComm.RPIComm;

public class CalibrateTargetAction extends Action {
	
	private double desiredX, desiredY;
	private double threshX, threshY;
	private double speedX, speedY;
	private AutoDriveAssembly autoDrive;
	private RPIComm rpiComm;
	
	public CalibrateTargetAction() {
		this.name = "<Calibrate Target Action>";	
		this.desiredX = 0.0;
		this.desiredY = 0.0;
		this.threshX = 0.0;
		this.threshY = 0.0;
		this.speedX = 0.0;
		this.speedY = 0.0;
		
		autoDrive = AutoDriveAssembly.GetInstance();		
		rpiComm = RPIComm.GetInstance();
	}
	
	public CalibrateTargetAction(String name, double desiredX, double desiredY, double threshX, double threshY, double speedX, double speedY)
	{
		this.name = name;
		this.desiredX = desiredX;
		this.desiredY = desiredY;
		this.threshX = threshX;
		this.threshY = threshY;
		this.speedX = speedX;
		this.speedY = speedY;
		
		autoDrive = AutoDriveAssembly.GetInstance();		
		rpiComm = RPIComm.GetInstance();
	}
	
	// action entry
	public void initialize() {
		
		// reset the RPi Vision Table
		rpiComm.autoInit();
						
		// set the desired target X and Y
		rpiComm.setDesired(desiredX, desiredY, threshX, threshY, speedX, speedY);
		
		rpiComm.setMovementModes(true, true);  // forward and lateral movement
				
		super.initialize();
	}
	
	// called periodically
	public void process()  {
		
		rpiComm.updateValues();
		
		if (rpiComm.hasTarget()) {
			
			rpiComm.targetProcessing();
			
			double leftVal = rpiComm.getLeftDriveValue();
			double rightVal = rpiComm.getRightDriveValue();
			
			autoDrive.drive(leftVal, rightVal, 0);
		}
		else {
			// no target - stop motors
			autoDrive.drive(0, 0, 0);
		}
		
		super.process();
	}
	
	// state cleanup and exit
	public void cleanup() {
		// do some calibrate cleanup
		autoDrive.drive(0, 0, 0);
					
		// cleanup base class
		super.cleanup();
	}
	
	public void persistWrite(int counter, Preferences prefs) {

		// create node for action
		Preferences actionPrefs = prefs.node(counter + "_" + this.name);
	
		// store action name
		actionPrefs.put("class",this.getClass().toString());
	}

}
