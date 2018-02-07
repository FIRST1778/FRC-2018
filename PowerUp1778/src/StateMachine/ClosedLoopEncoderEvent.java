package StateMachine;

import java.util.prefs.Preferences;

import Systems.CubeManagement;
import Systems.DriveAssembly;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Utility;

// event triggered when closed loop position control gets to within an error range of target for a time period
public class ClosedLoopEncoderEvent extends Event {
	
	private String name;
	private int targetEncoderTicks;
	private double errorThresholdTicks;
	private double durationSec;
	private long startTimeUs;
	
	public ClosedLoopEncoderEvent()
	{	
		this.name = "<Closed-Loop Encoder Event>";
		this.durationSec = 0.0;
		this.errorThresholdTicks = 0;
		this.targetEncoderTicks = 0;
		DriveAssembly.initialize();
	}
	
	public ClosedLoopEncoderEvent(int targetEncoderTicks, double errorThreshTicks, double durationSec)
	{
		this.name = "<Closed-Loop Encoder Event>";
		this.durationSec = durationSec;
		this.errorThresholdTicks = errorThreshTicks;
		this.targetEncoderTicks = targetEncoderTicks;
		DriveAssembly.initialize();
	}
	
	// overloaded initialize method
	public void initialize()
	{
		//System.out.println("ClosedLoopEncoderEvent initialized!");
		startTimeUs = RobotController.getFPGATime();
		
		super.initialize();
	}
	
	// overloaded trigger method
	public boolean isTriggered()
	{		
		// measure current position error
		int actualEncoderTicks = CubeManagement.getLiftPos();
		double errorEncoderTicks = Math.abs(targetEncoderTicks - actualEncoderTicks);
		if (errorEncoderTicks > errorThresholdTicks)
		{
			// outside error range...
			// reset timer
			startTimeUs = RobotController.getFPGATime();
			return false;
		}
	
		long currentTimeUs = RobotController.getFPGATime();
		double delta = (currentTimeUs - startTimeUs)/1e6;
		//System.out.println("delta = " + delta + " duration = " + durationSec);
		
		if (delta < durationSec)
		{
			// within error range, but not for enough time
			return false;
		}
		
		// within error range for enough time
		System.out.println("ClosedLoopEncoderEvent triggered!");
		return true;
	}
	
	public void persistWrite(int counter, Preferences prefs) {

		// create node for event
		Preferences eventPrefs = prefs.node(counter + "_" + this.name);
	
		// store event details
		eventPrefs.put("class",this.getClass().toString());
		eventPrefs.putDouble("durationSec",this.durationSec);		
	}
}
