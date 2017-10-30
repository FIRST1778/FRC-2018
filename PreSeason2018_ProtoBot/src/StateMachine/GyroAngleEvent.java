package StateMachine;

import java.util.prefs.Preferences;

import NetworkComm.InputOutputComm;
import Systems.NavXSensor;

// event triggered when gyro gets to a certain predetermined angle
public class GyroAngleEvent extends Event {
	
	private String name;
	
	// which side of the gyro angle determines the trigger
	public enum AnglePolarity { kGreaterThan, kLessThan };

	private double angleToTurn = 0.0;
	private double accuracyDeg = 5.0;
	private AnglePolarity polarity;
	private boolean resetGyro = true;
	private NavXSensor navX;
	private InputOutputComm ioComm;
	
	public GyroAngleEvent(double angleToTurn, boolean resetGyro, AnglePolarity polarity)
	{
		this.name = "<Gyro Angle Event>";
		
		this.angleToTurn = angleToTurn;
		this.resetGyro = resetGyro;
		this.polarity = polarity;
		
		navX = NavXSensor.GetInstance();
		ioComm = InputOutputComm.GetInstance();
	}
	
	// overloaded initialize method
	public void initialize()
	{
		//System.out.println("GyroAngleEvent initialized!");
		if (resetGyro)
			navX.reset();
		
		super.initialize();
	}
	
	private double getGyroAngle() {
		//double gyroAngle = 0.0;
		//double gyroAngle = navX.getYaw();  // -180 deg to +180 deg
		double gyroAngle = navX.getAngle();  // continuous angle (can be larger than 360 deg)
		
		//System.out.println("autoPeriodicStraight:  Gyro angle = " + gyroAngle);
			
		// send output data for test & debug
		ioComm.putBoolean(InputOutputComm.LogTable.kMainLog,"Auto/IMU_Connected",navX.isConnected());
		ioComm.putBoolean(InputOutputComm.LogTable.kMainLog,"Auto/IMU_Calibrating",navX.isCalibrating());

		//System.out.println("gyroAngle = " + gyroAngle);
		ioComm.putDouble(InputOutputComm.LogTable.kMainLog,"Auto/GyroAngle", gyroAngle);		

		return gyroAngle;
	}
	
	// overloaded trigger method
	public boolean isTriggered()
	{
		
		double gyroAngle = getGyroAngle();
		
		if (polarity == AnglePolarity.kGreaterThan) {
			// trigger only if angle is greater than target
			if ((gyroAngle - angleToTurn) > 0) {
				System.out.println("GyroAngleEvent triggered!");
				return true;
			}
		}
		else {
			// trigger only if angle is less than target
			if ((gyroAngle - angleToTurn) < 0) {
				System.out.println("GyroAngleEvent triggered!");
				return true;
			}			
		}
		
		return false;
	}
	
	public void persistWrite(int counter, Preferences prefs) {

		// create node for event
		Preferences eventPrefs = prefs.node(counter + "_" + this.name);
	
		// store event details
		eventPrefs.put("class",this.getClass().toString());
		eventPrefs.putDouble("angleToTurn", this.angleToTurn);
	}

}
