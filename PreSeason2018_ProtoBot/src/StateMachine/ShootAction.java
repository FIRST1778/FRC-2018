package StateMachine;

import Systems.BallManagement;

public class ShootAction extends Action {

	private int shooterStrength;
	private BallManagement ballCtrl;
	
	public ShootAction() {
		this.name = "<Shoot Action>";
		shooterStrength = BallManagement.MOTOR_MEDIUM;
		ballCtrl = BallManagement.GetInstance();
	}
	
	public ShootAction(String name, int shootStrength) {
		this.name = name;
		this.shooterStrength = shootStrength;
		ballCtrl = BallManagement.GetInstance();
	}
	
	// action entry
	public void initialize() {
		ballCtrl.setShooterStrength(shooterStrength);
		
		super.initialize();
	}
	
	public void process() {
		
		super.process();
	}
	
	public void cleanup() {
		ballCtrl.resetMotors();
		
		super.cleanup();
	}
}
