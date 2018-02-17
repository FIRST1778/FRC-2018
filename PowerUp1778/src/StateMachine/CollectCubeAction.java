package StateMachine;

import NetworkComm.InputOutputComm;
import Systems.CubeManagement;

public class CollectCubeAction extends Action {
	
	private String name;
	
	public CollectCubeAction()
	{
		this.name = "<Collect Cube Action>";	

		CubeManagement.initialize();
		InputOutputComm.initialize();
	}
	
	public CollectCubeAction(String name)
	{
		this.name = name;
		
		CubeManagement.initialize();		
		InputOutputComm.initialize();
	}
		
	// action entry
	public void initialize() {
		
		// do some clamp initialization, start the collector motors to collect the cube
		CubeManagement.collectCube();
		
		super.initialize();
	}
	
	// called periodically
	public void process()  {
		
		// do some stuff - nothing specific required for flipper
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
