package StateMachine;

import NetworkComm.InputOutputComm;
import Systems.CubeManagement;

public class DepositCubeAction extends Action {
	
	private String name;
	
	public DepositCubeAction()
	{
		this.name = "<Deposit Cube Action>";	

		CubeManagement.initialize();
		InputOutputComm.initialize();
	}
	
	public DepositCubeAction(String name)
	{
		this.name = name;
		
		CubeManagement.initialize();		
		InputOutputComm.initialize();
	}
		
	// action entry
	public void initialize() {
		
		// do some lift initialization, start the collector motors to deposit the cube
		CubeManagement.depositCube();
		
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
