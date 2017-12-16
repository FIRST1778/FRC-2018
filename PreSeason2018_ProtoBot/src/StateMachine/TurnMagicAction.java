package StateMachine;

import java.util.prefs.Preferences;

import Systems.AutoDriveAssembly;
import Systems.NavXSensor;

public class TurnMagicAction extends Action {
	//private double leftRevs = 0.0;
	//private double rightRevs = 0.0;
	private double leftPosInches = 0.0;
	private double rightPosInches = 0.0;
	private int speedToTurn = 0; 	
	private int accelToTurn = 0;
			
	public TurnMagicAction(double leftPosInches, double rightPosInches, int speed, int accel)
	{
		this.name = "<Turn Magic Action>";
		//this.leftRevs = leftPosInches * REVS_PER_INCH;
		//this.rightRevs = rightPosInches * REVS_PER_INCH;
		this.leftPosInches = leftPosInches;
		this.rightPosInches = rightPosInches;
		this.speedToTurn = speed;
		this.accelToTurn = accel;
				
		AutoDriveAssembly.initialize();
	}
	
	public TurnMagicAction(String name, double leftPosInches, double rightPosInches, int speed, int accel)
	{
		this.name =  name;
		//this.leftRevs = leftPosInches * REVS_PER_INCH;
		//this.rightRevs = rightPosInches * REVS_PER_INCH;
		this.leftPosInches = leftPosInches;
		this.rightPosInches = rightPosInches;
		this.speedToTurn = speed;
		this.accelToTurn = accel;
		
		AutoDriveAssembly.initialize();
	}
	
	// action entry
	public void initialize() {
				
		// initialize motor assembly for auto - use motion magic (closed loop control targets)
		AutoDriveAssembly.autoInit(true, 0.0, true);
		AutoDriveAssembly.autoMagicTurn(leftPosInches, rightPosInches, speedToTurn, accelToTurn);
		
		super.initialize();
	}
	
	// called periodically
	public void process()  {
				
		// PID motors driving toward target here - no action required
		super.process();
	}
	
	// action cleanup and exit
	public void cleanup() {
		// do some drivey cleanup
					
		AutoDriveAssembly.autoStop();
		
		// cleanup base class
		super.cleanup();
	}
	
	public void persistWrite(int counter, Preferences prefs) {

	}

}
