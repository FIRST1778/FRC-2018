package StateMachine;

import Systems.CameraControl;

public class CameraAction extends Action {
	
	public CameraAction() {
		this.name = "<Camera Action>";
		CameraControl.initialize();
	}
	
	public CameraAction(String name) {
		this.name = name;
		CameraControl.initialize();
	}
	
	// action entry
	public void initialize() {
		
		// in auto, turn on extra LEDs!
		CameraControl.setCameraLed(true);
		
		super.initialize();
	}
	
	public void process() {
		
		super.process();
	}
	
	public void cleanup() {
		
		super.cleanup();
	}

}
