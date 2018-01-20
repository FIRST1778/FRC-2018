package StateMachine;

import NetworkComm.InputOutputComm;
import Systems.CubeManagement;

public class FlipperAction extends Action {
	
	private String name;
	private boolean goUp = true;    // assumes lift starts up
	
	public FlipperAction(boolean goUp)
	{
		this.name = "<Flipper Action>";	
		this.goUp = goUp;

		CubeManagement.initialize();
		InputOutputComm.initialize();
	}
	
	public FlipperAction(String name, boolean goUp)
	{
		this.name = name;
		this.goUp = goUp;
		
		CubeManagement.initialize();		
		InputOutputComm.initialize();
	}
		
	// action entry
	public void initialize() {
		
		// do some lift initialization, start the lift
		if (goUp)
			CubeManagement.flipperUp();
		else
			CubeManagement.flipperDown();
				
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
