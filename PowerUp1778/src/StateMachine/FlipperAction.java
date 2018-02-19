package StateMachine;

import NetworkComm.InputOutputComm;
import Systems.CubeManagement;

public class FlipperAction extends Action {
	
	private String name;
	
	public FlipperAction()
	{
		this.name = "<Flipper Deploy Action>";	

		CubeManagement.initialize();
		InputOutputComm.initialize();
	}
	
	public FlipperAction(String name)
	{
		this.name = name;
		
		CubeManagement.initialize();		
		InputOutputComm.initialize();
	}
		
	// action entry
	public void initialize() {
		
		//CubeManagement.flipperDeploy();
				
		super.initialize();
	}
	
	// called periodically
	public void process()  {
		
		// do some stuff - nothing specific required for flipper
		super.process();
	}
	
	// state cleanup and exit
	public void cleanup() {
		// do some cleanup
					
		// cleanup base class
		super.cleanup();
	}
	
}
