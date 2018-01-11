package Systems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

/* deprecated 2018
import com.ctre.phoenix.MotorControl.SmartMotorController.FeedbackDevice;
import com.ctre.phoenix.MotorControl.SmartMotorController.FeedbackDeviceStatus;
import com.ctre.phoenix.MotorControl.SmartMotorController.TalonControlMode;
*/

import NetworkComm.InputOutputComm;
import Utility.HardwareIDs;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Spark;

public class CubeManagement {
	
	private static boolean initialized = false;
	
	private static boolean feeding = false;
		
	private static final double COLLECTOR_IN_LEVEL = -0.75;
	private static final double COLLECTOR_OUT_LEVEL = 0.75;

	private static final double LIFT_UP_LEVEL = -0.75;
	private static final double LIFT_DOWN_LEVEL = 0.75;
	
	private static final double DEAD_ZONE_THRESHOLD = 0.05;
		    
	public static void initialize() {
		
		if (initialized)
			return;
		
		InputOutputComm.initialize();
		
		// reset trigger init time
		initTriggerTime = RobotController.getFPGATime();

        // create and reset collector relay
		//collectorSolenoid = new Spark(HardwareIDs.COLLECTOR_SOLENOID_PWM_ID);
                
		// create motors & servos
		leftCollectorMotor = new TalonSRX(HardwareIDs.LEFT_COLLECTOR_TALON_ID);
		rightCollectorMotor = new TalonSRX(HardwareIDs.RIGHT_COLLECTOR_TALON_ID);
		
		liftMotor = new TalonSRX(HardwareIDs.LIFT_TALON_ID);
		
		
		// make sure all motors are off
		resetMotors();
		
		gamepad = new Joystick(HardwareIDs.GAMEPAD_ID);
		
		initialized = true;
	}
						
	// shooter and support motors
	private static TalonSRX leftCollectorMotor, rightCollectorMotor; 
	private static TalonSRX liftMotor;
		
	private static Joystick gamepad;
		
	// wait 0.25 s between button pushes on shooter
    private static final int TRIGGER_CYCLE_WAIT_US = 250000;
    private static double initTriggerTime;
    	
	public static void resetMotors()
	{		
		leftCollectorMotor.set(ControlMode.PercentOutput, 0);
		rightCollectorMotor.set(ControlMode.PercentOutput, 0);	
		liftMotor.set(ControlMode.PercentOutput, 0);
	}
	
		
	private static void checkCollectorControls() {
						
		// collector control
		double collectorLevel = gamepad.getRawAxis(HardwareIDs.COLLECTOR_IN_AXIS);
		if (Math.abs(collectorLevel) > DEAD_ZONE_THRESHOLD)
			collectorLevel = COLLECTOR_IN_LEVEL;
		else if (gamepad.getRawButton(HardwareIDs.COLLECTOR_OUT_BUTTON))
			collectorLevel = COLLECTOR_OUT_LEVEL;
		else
			collectorLevel = 0.0;
		InputOutputComm.putDouble(InputOutputComm.LogTable.kMainLog,"CubeMgmt/CollectorLevel", collectorLevel);
		leftCollectorMotor.set(ControlMode.PercentOutput, collectorLevel);
		rightCollectorMotor.set(ControlMode.PercentOutput, collectorLevel);
		
	}
	
	private static void checkLiftControls() {
		// cube lift control
		double liftLevel = gamepad.getRawAxis(HardwareIDs.LIFT_DOWN_AXIS);
		if (Math.abs(liftLevel) > DEAD_ZONE_THRESHOLD)
			liftLevel = LIFT_DOWN_LEVEL;
		else if (gamepad.getRawButton(HardwareIDs.LIFT_UP_BUTTON))
			liftLevel = LIFT_UP_LEVEL;
		else
			liftLevel = 0.0;
		InputOutputComm.putDouble(InputOutputComm.LogTable.kMainLog,"CubeMgmt/LiftLevel", liftLevel);
		liftMotor.set(ControlMode.PercentOutput, liftLevel);
	}
	
	public static void autoInit() {
				
		resetMotors();
		
        initTriggerTime = RobotController.getFPGATime();
	}

	public static void teleopInit() {
				
		resetMotors();
		
		// spawn a wait thread to turn relays back off after a number of seconds
		/*
		new Thread() {
			public void run() {
				try {
					Thread.sleep(3000);  // wait a number of sec before starting to feed
					gearTrayOff();	 	 // turn relays off
				} catch (Exception e) {
					e.printStackTrace();
				}
			}
		}.start();
		*/
		
        initTriggerTime = RobotController.getFPGATime();
        
	}
	
	public static void teleopPeriodic() {
		
		checkCollectorControls();
		checkLiftControls();

	}
	
}
