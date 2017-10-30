package StateMachine;

import NetworkComm.InputOutputComm;
import NetworkComm.RPIComm;

public class CalibratedEvent extends Event {
	private String name;
	private double desiredX, desiredY;
	private double marginX, marginY;
	private InputOutputComm ioComm;
	private RPIComm rpiComm;
	
	public CalibratedEvent()
	{	
		ioComm = InputOutputComm.GetInstance();
		rpiComm = RPIComm.GetInstance();
		
		this.name = "<Calibrated Event>";
		this.desiredX = 0.0;
		this.desiredY = 0.0;
		this.marginX = 0.0;
		this.marginY = 0.0;
	}
	
	public CalibratedEvent(double desiredX, double desiredY, double marginX, double marginY)
	{
		ioComm = InputOutputComm.GetInstance();

		this.name = "<Calibrated Event>";
		
		// desired location
		this.desiredX = desiredX;
		this.desiredY = desiredY;
		
		// acceptable error margin
		this.marginX = marginX;
		this.marginY = marginY;
	}
	
	// overloaded initialize method
	public void initialize()
	{		
		super.initialize();
	}
	
	// overloaded trigger method
	public boolean isTriggered()
	{
		if (!rpiComm.hasTarget())
			return false;
		
		// read network table target data
		
		double deltaX = Math.abs(desiredX - rpiComm.targetX);
		double deltaY = Math.abs(desiredY - rpiComm.targetY);
		
		ioComm.putDouble(InputOutputComm.LogTable.kMainLog,"Auto/desiredX", desiredX);		
		ioComm.putDouble(InputOutputComm.LogTable.kMainLog,"Auto/desiredY", desiredY);		
		ioComm.putDouble(InputOutputComm.LogTable.kMainLog,"Auto/deltaX", deltaX);		
		ioComm.putDouble(InputOutputComm.LogTable.kMainLog,"Auto/deltaY", deltaY);		
		
		if ((deltaX < marginX) && (deltaY < marginY))
		{
			System.out.println("CalibratedEvent triggered!");
			return true;
		}
		
		return false;
	}

}
