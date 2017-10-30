package NetworkComm;

import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.networktables.NetworkTable;  // deprecated in 2018
import edu.wpi.first.networktables.NetworkTable;

public class RPIComm {
        
	// comm objects
	private InputOutputComm ioComm;
	private  NetworkTable table;

	// camera image parameters
    private static int frameWidth = 160;
    private static int frameHeight = 120;
   	
	// Target accuracy thresholds
	private static final double X_THRESHOLD = 5;
	private static final int Y_THRESHOLD = 5;
	private static final double AREA_THRESHOLD = 20;

	// singleton class elements (ensures only one instance of this class)
	private static final RPIComm instance = new RPIComm();
    
	private RPIComm() {

		ioComm = InputOutputComm.GetInstance();
		
		//table = NetworkTable.getTable("InputOutput1778/DataTable");
        table = new NetworkTable(null, "InputOutput1778/DataTable");
        
		table.getEntry("autoCam").setBoolean(false);
		
		threshX = X_THRESHOLD;
		threshY = Y_THRESHOLD;
		speedX = DRIVE_SPEED_X;
		speedY = DRIVE_SPEED_Y;
	}
		
	public static RPIComm GetInstance() {
		return instance;
	}

    // instance data and methods
    private boolean lateralMovement = true;
    private boolean forwardMovement = false;
        
	public boolean targetCentered = false;
	
	public double numTargets, targetX, targetY, targetArea, targetDistance;
	private double desiredX, desiredY;
	private double threshX, threshY;
	private double speedX, speedY;
	
	// Robot drive output
	private double driveLeft;
	private double driveRight;
	
	// Robot targeting speed (% how fast it moves and turns)
	private final double DRIVE_SPEED_X = 0.4;
	private final double DRIVE_SPEED_Y = 0.4;
	
	// Number of loops to perform to guarantee the robot is lined up with the target
	private final int IS_CENTERED_DELAY = 15;
	private int readyTimer = 0;
	    
    public void setMovementModes(boolean forwardFlag, boolean lateralFlag) {
    	forwardMovement = forwardFlag;
    	lateralMovement = lateralFlag;
    }
    
    public void setDesired(double x, double y, double tX, double tY, double spX, double spY)
    {
    	desiredX = x;
    	desiredY = y;
    	threshX = tX;
    	threshY = tY;
    	speedX = spX;
    	speedY = spY;
    }
    
    public void autoInit() {
    	numTargets = 0;
    	desiredX = frameWidth/2;
    	desiredY = frameHeight/2;
    	targetX = frameWidth/2;
    	targetY = frameHeight/2;
    	targetArea = 0;
    	targetDistance = 0;
    	
		reset();
		
		//table.putBoolean("autoCam", true);
		table.getEntry("autoCam").setBoolean(false);  // keep camera auto exposure on with RPi
    }
    
    public void teleopInit() {
    	numTargets = 0;
    	desiredX = frameWidth/2;
    	desiredY = frameHeight/2;
    	targetX = frameWidth/2;
    	targetY = frameHeight/2;
    	targetArea = 0;
    	targetDistance = 0;
    	
		reset();
		
		table.getEntry("autoCam").setBoolean(false);
    }
    
    public void disabledInit() {
    	
		table.getEntry("autoCam").setBoolean(false);
    }
  
        
    public void reset() {
		
		driveLeft = 0;
		driveRight = 0;
		
		readyTimer = 0;

		targetCentered = false;
    }
    
    public void updateValues() {
            	
    	// Default data if network table data pull fails
		double defaultDoubleVal = 0.0;
		
		// Pull data from grip
		numTargets = table.getEntry("targets").getDouble(defaultDoubleVal);
		targetX = table.getEntry("targetX").getDouble(defaultDoubleVal);
		targetY = table.getEntry("targetY").getDouble(defaultDoubleVal);
		targetArea = table.getEntry("targetArea").getDouble(defaultDoubleVal);
		targetDistance = table.getEntry("targetDistance").getDouble(defaultDoubleVal);
		
		Timer.delay(0.02);
    }
    
    public void targetProcessing() {
		if (numTargets > 0) {
			
			// Debug only - print out values read from network table
			//System.out.println("Time_ms= " + System.currentTimeMillis() + " targets = " + numTargets + ", delta = (" + deltaX + ", " + deltaY + ")");
			double deltaX = targetX - desiredX;
			double deltaY = targetY - desiredY;
			
			// do something with position information
	    	// if a valid target exists (one that meets filter criteria)
				
			// First, focus on centering X!		
			// neg delta X = actual left of goal, turn LEFT => angular velocity = NEG
			// pos delta X = actual right of goal, turn RIGHT => angular velocity = POS
			// if no lateral movement needed, pass through
			
			if ((Math.abs(deltaX) < threshX) || (!lateralMovement))  {
				// X is now centered!  Next focus on Y!
				// neg delta Y = actual above goal, move backward => forward velocity = NEG
				// pos delta Y = actual below goal, move forward => forward velocity = POS
				// if no forward movement needed, pass through
				
				if ((Math.abs(deltaY) < threshY) || (!forwardMovement))  {
					// Both X and Y are centered!
					driveLeft = 0;
					driveRight = 0;
					readyTimer++;
									
					// if we continued to be centered for a reasonable number of iterations
					if(readyTimer >= IS_CENTERED_DELAY) {
						//System.out.println("NetworkCommAssembly: TARGET CENTERED!.... X: " + deltaX + " Y: " + deltaY + 
						//		"driveLeft = " + driveLeft +
						//		"driveRight = " + driveRight);
						String outputStr = String.format("RPIComm: TARGET CENTERED!.... X: %.1f Y: %.1f driveLeft= %.1f driveRight= %.1f",
											deltaX, deltaY,driveLeft,driveRight);
						ioComm.putString(InputOutputComm.LogTable.kRPICommLog,"RPIComm",outputStr);
						targetCentered = true;
					}
					return;
				}
				else {
					// Set the left and right motors to help center Y
					driveLeft = Math.copySign(speedY, deltaY);
					driveRight = Math.copySign(speedY, deltaY);
					targetCentered = false;
					readyTimer = 0;

					String outputStr = String.format("RPIComm: CENTERING Y.... X: %.1f Y: %.1f driveLeft= %.1f driveRight= %.1f",
							deltaX, deltaY,driveLeft,driveRight);
					ioComm.putString(InputOutputComm.LogTable.kRPICommLog,"RPIComm",outputStr);
					return;
				}
			}
			else {
				// Set the left and right motors to help center X
				driveLeft = Math.copySign(speedX, deltaX);
				driveRight = Math.copySign(speedX, -deltaX);
				targetCentered = false;
				readyTimer = 0;
				
				String outputStr = String.format("RPIComm: CENTERING X.... X: %.1f Y: %.1f driveLeft= %.1f driveRight= %.1f",
						deltaX, deltaY,driveLeft,driveRight);
				ioComm.putString(InputOutputComm.LogTable.kRPICommLog,"RPIComm",outputStr);
				return;
			}		
		}
		else {
			// no target found!  Reset targeting params
			ioComm.putString(InputOutputComm.LogTable.kRPICommLog,"RPIComm","No target found");
			reset();
		}
			
		Timer.delay(0.02);
    }
    
	// Returns the value for the left side drivetrain
	public double getLeftDriveValue() {
		return driveLeft;
	}
	
	// Returns the value for the right side drivetrain
	public double getRightDriveValue() {
		return driveRight;
	}
	
	// Returns true if the target is visible, returns false otherwise
	public boolean hasTarget() {
		return (numTargets > 0);
	}
	
	public double getFrameWidth() {
		return frameWidth;
	}
	
	public double getFrameHeight() {
		return frameHeight;
	}
	
	// returns delta from current desired X position
	public double getDeltaX() {
		if (!hasTarget())
			return 0;
		
		return (targetX - desiredX);
	}
	
	// returns delta from current desired Y position
	public double getDeltaY() {
		if (!hasTarget())
			return 0;
		
		return (targetY - desiredY);
	}
	
	// Returns true if the catapult is ready to shoot, returns false otherwise
	public boolean targetCentered() {
		return targetCentered;
	}
    
}
