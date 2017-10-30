
package Systems;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;

public class NavXSensor {
	
    // singleton class elements (ensures only one instance of this class)
	private static final NavXSensor instance = new NavXSensor();
    
	private NavXSensor() {
		System.out.println("NavXSensor constructor called...");
		
		try {
			ahrs = new AHRS(SPI.Port.kMXP);     
		} catch (RuntimeException ex ) {
            DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
        }

		reset();
	}
		
	public static NavXSensor GetInstance() {
		return instance;
	}
	
	// instance data and methods
	private AHRS ahrs;
	
	private double yawOffset = 0.0;
	
	public class Angles {
		float roll = 0f;
		float pitch = 0f;
		float yaw = 0f;
	}
				
	public void reset()
	{
		System.out.println("NavXSensor::reset called!");
		
		if (ahrs != null) 
		{
			ahrs.reset();
			ahrs.resetDisplacement();
			ahrs.zeroYaw();
			
			// allow zeroing to take effect
			Timer.delay(0.1);
			
			// get the absolute angle after reset - Not sure why it is non-zero, but we need to record it to zero it out
			yawOffset = ahrs.getAngle();	
			System.out.println("yawOffset read = " + yawOffset);
		}
	}

	public AHRS getAHRS() {		
		return ahrs;
	}
	
	public boolean isConnected() {
		if (ahrs != null) {
			return ahrs.isConnected();
		}
		
		return false;
	}
	
	public boolean isCalibrating() {
		if (ahrs != null) {
			return ahrs.isCalibrating();
		}
		
		return false;
	}
	
	public Angles getAngles()
	{
		Angles angles = new Angles();
		
		if (ahrs != null) {
			angles.roll = ahrs.getRoll();	
			angles.pitch = ahrs.getPitch();	
			angles.yaw = ahrs.getYaw();	
		}			
		
		return angles;
	}
	
	// returns yaw angle (-180 deg to +180 deg)
	public float getYaw() 
	{
		float yaw = 0f;
		
		if (ahrs != null) {
			yaw = ahrs.getYaw();	
		}			
		
		return yaw;
		
	}
	
	// returns absolute yaw angle (can be larger than 360 deg)
	public double getAngle() 
	{
		double yaw = 0f;
		
		if (ahrs != null) {
			yaw = ahrs.getAngle();	
			yaw -= yawOffset;  // needed to get to true angle
		}			
		
		return yaw;
		
	}
	
}
