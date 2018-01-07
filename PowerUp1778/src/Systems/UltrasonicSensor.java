package Systems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Ultrasonic;

import com.kauailabs.navx.frc.AHRS;

import Utility.HardwareIDs;

public class UltrasonicSensor {
		
	private static boolean initialized = false;
	
	public static void initialize() {
		if (initialized)
			return;
		
		ultrasonicDevice = new Ultrasonic(HardwareIDs.TRIGGER_CHANNEL_ID,HardwareIDs.ECHO_CHANNEL_ID);
		ultrasonicDevice.setAutomaticMode(true);
		
		initialized = true;
	}
			
	private static Ultrasonic ultrasonicDevice;
	
	public static void autoInit()
	{		
		ultrasonicDevice.setEnabled(true);
	}
	
	public static double getRange() {				
		return ultrasonicDevice.getRangeInches();
	}
}
