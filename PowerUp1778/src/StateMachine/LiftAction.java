package StateMachine;

import NetworkComm.InputOutputComm;
import Systems.CubeManagement;

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
		
		// do some lift initialization, start the lift
		CubeManagement.goToHeight(targetliftLevel);
				
		super.initialize();
	}
	
	// called periodically
	public void process()  {
		
		// do some lift stuff
		super.process();
	}
	
	// state cleanup and exit
	public void cleanup() {
		// do some drivey cleanup
					
		CubeManagement.autoStop();
		
		// cleanup base class
		super.cleanup();
	}
	
}
