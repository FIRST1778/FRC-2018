package Systems;

import com.kauailabs.navx.frc.AHRS;

import NetworkComm.InputOutputComm;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;

public class TurnController {

    // singleton class elements (ensures only one instance of this class)
	private static final TurnController instance = new TurnController();
    
	private TurnController() {
		navX = NavXSensor.GetInstance();	
		ahrs = navX.getAHRS();
		
		pidOut = new TurnOutput();

		pidCtrl = new PIDController(kP,kI,kD,kF,ahrs,pidOut);
		
		pidCtrl.setInputRange(-180.0, 180.0);
		pidCtrl.setOutputRange(-maxSpeed, maxSpeed);
		//pidCtrl.setAbsoluteTolerance(0.5);
		pidCtrl.setContinuous(true);
	}
		
	public static TurnController GetInstance() {
		return instance;
	}
	
	// instance data and methods
	private NavXSensor navX;
	private PIDController pidCtrl;
	
	// comp.bot - tuned 7/20/2017
	private static final double kP = 0.075;
	private static final double kI = 0.0;   // I not needed for PID position control
	private static final double kD = 0.14;
	private static final double kF = 0;     // F not needed for PID position control
		
	// proto.bot - tuned version
	//private static final double kP = 0.04;
	//private static final double kI = 0;     // I not needed for PID position control
	//private static final double kD = 0.175;
	//private static final double kF = 0;     // F not needed for PID position control
	
	private static final double maxSpeed = 0.5;
	
	private double angleTargetDeg = 0.0;	
	private TurnOutput pidOut;
	private AHRS ahrs;
	
	public void setAngle(double angleDeg) {
		
		angleTargetDeg = angleDeg;
		pidCtrl.setOutputRange(-maxSpeed, maxSpeed);
		pidCtrl.setSetpoint(angleTargetDeg);
	}

	public void setAngle(double angleDeg, double speed) {
		
		angleTargetDeg = angleDeg;
		pidCtrl.setOutputRange(-speed, speed);
		pidCtrl.setSetpoint(angleTargetDeg);
	}
	
	public void reset() {
		disable();
		navX.reset();
		angleTargetDeg = 0.0;
	}
	
	public void enable() {
		pidCtrl.enable();
	}
	
	public void disable() {
		pidCtrl.disable();
	}
	
	public double getLeft() {
		return pidOut.getValue();
	}
	
	public double getRight() {
		return -pidOut.getValue();
	}
	
	public class TurnOutput implements PIDOutput {
		private double myOutput = 0.0;
		
		public double getValue() {
			return myOutput;
		}
		
		public void pidWrite(double output) {
			myOutput = output;
		}
	}	
}
