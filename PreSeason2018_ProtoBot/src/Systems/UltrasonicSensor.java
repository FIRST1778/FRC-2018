package Systems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Ultrasonic;

import com.kauailabs.navx.frc.AHRS;

import Utility.HardwareIDs;

public class UltrasonicSensor {
		
    // singleton class elements (ensures only one instance of this class)
	private static final UltrasonicSensor instance = new UltrasonicSensor();
    
	private UltrasonicSensor() {
		ultrasonicDevice = new Ultrasonic(HardwareIDs.TRIGGER_CHANNEL_ID,HardwareIDs.ECHO_CHANNEL_ID);
		ultrasonicDevice.setAutomaticMode(true);
	}
		
	public static UltrasonicSensor GetInstance() {
		return instance;
	}
	
	private Ultrasonic ultrasonicDevice;
	
	public void autoInit()
	{		
		ultrasonicDevice.setEnabled(true);
	}
	
	public double getRange() {				
		return ultrasonicDevice.getRangeInches();
	}
}
