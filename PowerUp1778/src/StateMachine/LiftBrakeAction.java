package StateMachine;

import NetworkComm.InputOutputComm;
import Systems.CubeManagement;

public class LiftBrakeAction extends Action {
	
	private String name;
	private boolean brakeOn = true;    // assumes brake turn on
	
	public LiftBrakeAction(boolean brakeOn)
	{
		this.name = "<Lift Brake Action>";	
		this.brakeOn = brakeOn;

		CubeManagement.initialize();
		InputOutputComm.initialize();
	}
	
	public LiftBrakeAction(String name, boolean brakeOn)
	{
		this.name = name;
		this.brakeOn = brakeOn;
		
		CubeManagement.initialize();		
		InputOutputComm.initialize();
	}
		
	// action entry
	public void initialize() {
		
		// do some lift initialization, start the lift
		if (brakeOn)
			CubeManagement.liftBrakeOn();
		else
			CubeManagement.liftBrakeOff();
				
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
