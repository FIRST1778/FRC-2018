package StateMachine;

import Systems.CameraControl;

public class CameraAction extends Action {
	
	private double camPos = CameraControl.GEAR_CAM_POS;
	private CameraControl camCtrl;
	
	public CameraAction() {
		this.name = "<Camera Action>";
		camCtrl = CameraControl.GetInstance();
	}
	
	public CameraAction(String name, double camPos) {
		this.name = name;
		this.camPos = camPos;
		camCtrl = CameraControl.GetInstance();
	}
	
	// action entry
	public void initialize() {
		camCtrl.moveToPos(camPos);
		
		// in auto, turn on extra LEDs!
		camCtrl.setCameraLed(true);
		
		super.initialize();
	}
	
	public void process() {
		
		super.process();
	}
	
	public void cleanup() {
		
		super.cleanup();
	}

}
