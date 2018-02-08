package StateMachine;

import NetworkComm.InputOutputComm;
import Systems.CubeManagement;
import edu.wpi.first.wpilibj.Timer;

public class LiftAction extends Action {
	
	private String name;
	private int targetliftLevel = CubeManagement.BASE_LEVEL;
	
	public LiftAction(int targetLiftLevel)
	{
		this.name = "<Lift Action>";		
		this.targetliftLevel = targetLiftLevel;

		CubeManagement.initialize();
		InputOutputComm.initialize();
	}
	
	public LiftAction(String name, int targetliftLevel)
	{
		this.name =  name;
		this.targetliftLevel = targetliftLevel;
		
		CubeManagement.initialize();		
		InputOutputComm.initialize();
	}
		
	// action entry
	public void initialize() {
		
		// First turn off brake
		CubeManagement.liftBrakeOff();
		
		// Wait a fraction of a second for brake to disengage
		Timer.delay(0.2);
		
		// do some lift initialization, start the lift
		CubeManagement.goToTarget(targetliftLevel);
				
		super.initialize();
	}
	
	// called periodically
	public void process()  {
		
		// do some lift stuff
		super.process();
	}
	
	// state cleanup and exit
	public void cleanup() {
		
		// turn brake back on
		CubeManagement.liftBrakeOn();
		
		// Wait a fraction of a second for brake to engage
		Timer.delay(0.2);
					
		// cleanup base class
		super.cleanup();
	}
	
}
