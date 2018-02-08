package StateMachine;

import java.util.ArrayList;

import Systems.CubeManagement;

public class AutoNetworkBuilder {
		
	public final static int DO_NOTHING = 0;
	public final static int DRIVE_FORWARD = 1;
	
	// left position activities
	public final static int DEPOSIT_CUBE_SWITCH_LEFT = 2;
	public final static int DEPOSIT_CUBE_SCALE_LEFT = 3;
	public final static int DEPOSIT_CUBE_SCALE_RIGHT_FROM_LEFT = 4;
	public final static int MOVE_TO_SCALE_RIGHT_FROM_LEFT = 5;
	
	// center position activities
	public final static int DEPOSIT_CUBE_SWITCH_CENTER_LEFT = 6;
	public final static int DEPOSIT_CUBE_SWITCH_CENTER_RIGHT = 7;
	
	// right position activities
	public final static int DEPOSIT_CUBE_SWITCH_RIGHT = 8;
	public final static int DEPOSIT_CUBE_SCALE_RIGHT = 9;
	public final static int DEPOSIT_CUBE_SCALE_LEFT_FROM_RIGHT = 10;
	public final static int MOVE_TO_SCALE_LEFT_FROM_RIGHT = 11;

	// debug networks
	public final static int LIFT_FOREVER = 12;
	public final static int TURN_FOREVER = 13;
	public final static int PACE_FOREVER = 14;
	public final static int TURN_ONCE = 15;

	// closed-loop position cruise velocity and acceleration (used for all closed-loop position control)
	// units are RPM
	
	// ~4.5 ft/s - FAST
	private final static int CLOSED_LOOP_VEL_FAST = 2000;
	private final static int CLOSED_LOOP_ACCEL_FAST = 600;

	// ~2 ft/s - SLOW
	private final static int CLOSED_LOOP_VEL_SLOW = 800;
	private final static int CLOSED_LOOP_ACCEL_SLOW = 400;
	
	private static ArrayList<AutoNetwork> autoNets;
	
	private static boolean initialized = false;
		
	public static void initialize() {
		
		if (!initialized) {
			autoNets = null;
			
			initialized = true;
		}
	}
	
	public static ArrayList<AutoNetwork> readInNetworks() {
		
		if (!initialized)
			initialize();
		
		autoNets = new ArrayList<AutoNetwork>();
							
		// create networks
		autoNets.add(DO_NOTHING, createDoNothingNetwork());	
		autoNets.add(DRIVE_FORWARD, createDriveForward());	

		autoNets.add(DEPOSIT_CUBE_SWITCH_LEFT, createDepositCubeSwitchLeft());	
		autoNets.add(DEPOSIT_CUBE_SCALE_LEFT, createDepositCubeScaleLeft());	
		autoNets.add(DEPOSIT_CUBE_SCALE_RIGHT_FROM_LEFT, createDepositCubeScaleRightFromLeft());	
		autoNets.add(MOVE_TO_SCALE_RIGHT_FROM_LEFT, createMoveToScaleRightFromLeft());	
		
		autoNets.add(DEPOSIT_CUBE_SWITCH_CENTER_LEFT, createDepositCubeSwitchCenterLeft());	
		autoNets.add(DEPOSIT_CUBE_SWITCH_CENTER_RIGHT, createDepositCubeSwitchCenterRight());	
		
		autoNets.add(DEPOSIT_CUBE_SWITCH_RIGHT, createDepositCubeSwitchRight());	
		autoNets.add(DEPOSIT_CUBE_SCALE_RIGHT, createDepositCubeScaleRight());	
		autoNets.add(DEPOSIT_CUBE_SCALE_LEFT_FROM_RIGHT, createDepositCubeScaleLeftFromRight());	
		autoNets.add(MOVE_TO_SCALE_LEFT_FROM_RIGHT, createMoveToScaleLeftFromRight());	

		autoNets.add(LIFT_FOREVER, createLiftingForeverNetwork());	
		autoNets.add(TURN_FOREVER, createTurningForeverNetwork());	
		autoNets.add(PACE_FOREVER, createPacingForeverNetwork());	
		autoNets.add(TURN_ONCE, createTurningOnceNetwork());	

		return autoNets;
	}
	
	///////////////////////////////////////////////////////////
	/*            AutoState Creation Methods                 */
	/*            (These are used repeatedly)                */
	///////////////////////////////////////////////////////////
	
	private static AutoState createIdleState(String state_name)
	{
		AutoState idleState = new AutoState(state_name);
		IdleAction deadEnd = new IdleAction("<Dead End Action>");
		DriveForwardAction driveForwardReset = new DriveForwardAction("<Drive Forward Action -reset>", 0.0, true, 0.0);  // reset gyro
		idleState.addAction(deadEnd);
		idleState.addAction(driveForwardReset);
		
		return idleState;
	}
		
	private static AutoState createMagicDriveState(String state_name, double dist_inches, int max_vel_rpm, int max_accel_rpm) 
	{		
		AutoState driveState = new AutoState(state_name);
		DriveForwardMagicAction driveForwardMagicAction = new DriveForwardMagicAction("<Drive Forward Magic Action>", dist_inches, max_vel_rpm, max_accel_rpm, true, 0.0);
		//TimeEvent timer = new TimeEvent(2.5);  // drive forward timer event - allow PID time to settle
		ClosedLoopPositionEvent pos = new ClosedLoopPositionEvent(dist_inches, 0.5, 1.0);
		driveState.addAction(driveForwardMagicAction);
		//driveState.addEvent(timer);
		driveState.addEvent(pos);
		
		return driveState;
	}

	private static AutoState createMagicTurnState(String state_name, double angle_deg, double error_deg, double percent_vbus)
	{
		AutoState turnState = new AutoState(state_name);
		TurnPIDAction turnPidAction = new TurnPIDAction("<Turn PID action>", angle_deg, percent_vbus, true);
		//TimeEvent timer = new TimeEvent(2.5);  // timer event - allow PID time to settle
		ClosedLoopAngleEvent angle = new ClosedLoopAngleEvent(angle_deg,error_deg, 0.75);
		turnState.addAction(turnPidAction);
		//turnState.addEvent(timer);
		turnState.addEvent(angle);
		
		return turnState;
	}
	
	private static AutoState createFlipperState(String state_name, boolean flipperUp)
	{
		AutoState flipperState = new AutoState(state_name);
		FlipperAction flipperAction = new FlipperAction("<Flipper Action>", flipperUp);
		TimeEvent timer = new TimeEvent(0.1);  // flipper timer event
		flipperState.addAction(flipperAction);
		flipperState.addEvent(timer);
		
		return flipperState;
	}

	private static AutoState createLiftState(String state_name, int lift_level)
	{
		AutoState liftState = new AutoState(state_name);
		LiftAction liftAction = new LiftAction("<Lift Action>", lift_level);
		//TimeEvent timer5b = new TimeEvent(1.0);  // lift timer event
		ClosedLoopEncoderEvent enc = new ClosedLoopEncoderEvent(lift_level, 10, 1.0);
		liftState.addAction(liftAction);
		//liftState.addEvent(timer5b);
		liftState.addEvent(enc);
		
		return liftState;
	}
	
	private static AutoState createCubeDepositState(String state_name, double secs)
	{
		AutoState depositCubeState = new AutoState(state_name);
		DepositCubeAction deposit1 = new DepositCubeAction("<Deposit Cube Action>");
		TimeEvent timer = new TimeEvent(secs);  // deposit cube timer event
		depositCubeState.addAction(deposit1);
		depositCubeState.addEvent(timer);

		return depositCubeState;
	}

	////////////////////////////////////////////////////////////


	// **** DO NOTHING Network ***** 
	private static AutoNetwork createDoNothingNetwork() {
		
		AutoNetwork autoNet = new AutoNetwork("<Do Nothing Network>");
		
		AutoState idleState = new AutoState("<Idle State>");
		IdleAction deadEnd = new IdleAction("<Dead End Action>");
		idleState.addAction(deadEnd);

		autoNet.addState(idleState);	
		
		return autoNet;
	}

	// **** MOVE FORWARD Network ***** 
	// 1) drive forward for a number of sec
	// 2) go back to idle and stay there 
	private static AutoNetwork createDriveForward() {
		
		AutoNetwork autoNet = new AutoNetwork("<Drive Forward Network>");
			
		// create states
		AutoState driveState = createMagicDriveState("<Drive State 1>", 84.0, CLOSED_LOOP_VEL_FAST, CLOSED_LOOP_ACCEL_FAST);
		AutoState idleState = createIdleState("<Idle State>");

		// connect the state sequence
		driveState.associateNextState(idleState);
						
		// add states to the network list
		autoNet.addState(driveState);
		autoNet.addState(idleState);
				
		return autoNet;
	}

	// **** DEPOSIT CUBE SWITCH LEFT SIDE Network ***** 
	// 1) drive forward for a number of sec
	// 2) Turn RIGHT a number of degrees
	// 3) flipper down
	// 4) raise lift
	// 5) drive forward
	// 6) deposit cube
	// 7) go back to idle and stay there 
	private static AutoNetwork createDepositCubeSwitchLeft() {
		
		AutoNetwork autoNet = new AutoNetwork("<Deposit Cube Switch (left side) Network>");
			
		// create states
		AutoState driveState = createMagicDriveState("<Drive State 1>", 144.0, CLOSED_LOOP_VEL_FAST, CLOSED_LOOP_ACCEL_FAST);
		AutoState turnRightState = createMagicTurnState("<Turn Right State>", 90.0, 1.0, 0.6);
		AutoState flipperDownState = createFlipperState("<Flipper Down State>", false);
		AutoState liftUpState = createLiftState("<Lift Up State>", CubeManagement.SWITCH_LEVEL);
		AutoState driveState2 = createMagicDriveState("<Drive State 2>", 36.0, CLOSED_LOOP_VEL_SLOW, CLOSED_LOOP_ACCEL_SLOW);
		AutoState depositCubeState = createCubeDepositState("<Deposit Cube State>", 3.0);
		AutoState idleState = createIdleState("<Idle State>");
		
		// connect the state sequence
		driveState.associateNextState(turnRightState);
		turnRightState.associateNextState(flipperDownState);
		flipperDownState.associateNextState(liftUpState);
		liftUpState.associateNextState(driveState2);
		driveState2.associateNextState(depositCubeState);
		depositCubeState.associateNextState(idleState);
				
		// add states to the network list
		autoNet.addState(driveState);
		autoNet.addState(turnRightState);
		autoNet.addState(flipperDownState);
		autoNet.addState(liftUpState);
		autoNet.addState(driveState2);
		autoNet.addState(depositCubeState);
		autoNet.addState(idleState);
				
		return autoNet;
	}

	// **** DEPOSIT CUBE SCALE LEFT SIDE Network ***** 
	// 1) drive forward for a number of sec
	// 2) Turn RIGHT a number of degrees
	// 3) flipper down
	// 4) raise lift
	// 5) drive forward
	// 6) deposit cube
	// 7) go back to idle and stay there 
	private static AutoNetwork createDepositCubeScaleLeft() {
		
		AutoNetwork autoNet = new AutoNetwork("<Deposit Cube Scale (left side) Network>");
		
		// create states
		AutoState driveState = createMagicDriveState("<Drive State 1>", 302.0, CLOSED_LOOP_VEL_FAST, CLOSED_LOOP_ACCEL_FAST);
		AutoState turnRightState = createMagicTurnState("<Turn Right State>", 90.0, 1.0, 0.6);
		AutoState flipperDownState = createFlipperState("<Flipper Down State>", false);
		AutoState liftUpState = createLiftState("<Lift Up State>", CubeManagement.SCALE_LEVEL);
		AutoState driveState2 = createMagicDriveState("<Drive State 2>", 24.0, CLOSED_LOOP_VEL_SLOW, CLOSED_LOOP_ACCEL_SLOW);
		AutoState depositCubeState = createCubeDepositState("<Deposit Cube State>", 3.0);
		AutoState idleState = createIdleState("<Idle State>");
			
		// connect the state sequence
		driveState.associateNextState(turnRightState);
		turnRightState.associateNextState(flipperDownState);
		flipperDownState.associateNextState(liftUpState);
		liftUpState.associateNextState(driveState2);
		driveState2.associateNextState(depositCubeState);
		depositCubeState.associateNextState(idleState);
						
		// add states to the network list
		autoNet.addState(driveState);
		autoNet.addState(turnRightState);
		autoNet.addState(flipperDownState);
		autoNet.addState(liftUpState);
		autoNet.addState(driveState2);
		autoNet.addState(depositCubeState);
		autoNet.addState(idleState);
				
		return autoNet;
	}
	
	// **** DEPOSIT CUBE SCALE RIGHT FROM LEFT SIDE Network ***** 
	// 1) drive forward for a number of sec
	// 2) Turn RIGHT a number of degrees
	// 3) drive forward for a number of sec
	// 4) Turn LEFT a number of degrees
	// 5) flipper down
	// 6) raise lift
	// 7) drive forward
	// 8) deposit cube
	// 9) go back to idle and stay there 
	private static AutoNetwork createDepositCubeScaleRightFromLeft() {
		
		AutoNetwork autoNet = new AutoNetwork("<Deposit Cube Scale (right from left side) Network>");
				
		// create states
		AutoState driveState = createMagicDriveState("<Drive State 1>", 200.0, CLOSED_LOOP_VEL_FAST, CLOSED_LOOP_ACCEL_FAST);
		AutoState turnRightState = createMagicTurnState("<Turn Right State>", 90.0, 1.0, 0.6);
		AutoState driveState2 = createMagicDriveState("<Drive State 2>", 256.0, CLOSED_LOOP_VEL_FAST, CLOSED_LOOP_ACCEL_FAST);
		AutoState turnLeftState = createMagicTurnState("<Turn Left State>", -135.0, 1.0, 0.6);
		AutoState flipperDownState = createFlipperState("<Flipper Down State>", false);
		AutoState liftUpState = createLiftState("<Lift Up State>", CubeManagement.SCALE_LEVEL);
		AutoState driveState3 = createMagicDriveState("<Drive State 3>", 36.0, CLOSED_LOOP_VEL_SLOW, CLOSED_LOOP_ACCEL_SLOW);
		AutoState depositCubeState = createCubeDepositState("<Deposit Cube State>", 3.0);
		AutoState idleState = createIdleState("<Idle State>");

		// connect the state sequence
		driveState.associateNextState(turnRightState);
		turnRightState.associateNextState(driveState2);
		driveState2.associateNextState(turnLeftState);
		turnLeftState.associateNextState(flipperDownState);
		flipperDownState.associateNextState(liftUpState);
		liftUpState.associateNextState(driveState3);
		driveState3.associateNextState(depositCubeState);
		depositCubeState.associateNextState(idleState);
						
		// add states to the network list
		autoNet.addState(driveState);
		autoNet.addState(turnRightState);
		autoNet.addState(driveState2);
		autoNet.addState(turnLeftState);
		autoNet.addState(flipperDownState);
		autoNet.addState(liftUpState);
		autoNet.addState(driveState3);
		autoNet.addState(depositCubeState);
		autoNet.addState(idleState);
				
		return autoNet;
	}

	// **** MOVE TO SCALE RIGHT FROM LEFT SIDE Network ***** 
	// 1) drive forward for a number of sec
	// 2) Turn RIGHT a number of degrees
	// 3) drive forward
	// 4) flipper down
	// 5) raise lift
	// 6) go back to idle and stay there 
	private static AutoNetwork createMoveToScaleRightFromLeft() {
		
		AutoNetwork autoNet = new AutoNetwork("<Move to Scale Right (left side) Network>");
				
		// create states
		AutoState driveState = createMagicDriveState("<Drive State 1>", 200.0, CLOSED_LOOP_VEL_FAST, CLOSED_LOOP_ACCEL_FAST);
		AutoState turnRightState = createMagicTurnState("<Turn Right State>", 90.0, 1.0, 0.6);
		AutoState driveState2 = createMagicDriveState("<Drive State 2>", 192.0, CLOSED_LOOP_VEL_FAST, CLOSED_LOOP_ACCEL_FAST);
		AutoState flipperDownState = createFlipperState("<Flipper Down State>", false);
		AutoState liftUpState = createLiftState("<Lift Up State>", CubeManagement.SCALE_LEVEL);
		AutoState idleState = createIdleState("<Idle State>");
		
		// connect the state sequence
		driveState.associateNextState(turnRightState);
		turnRightState.associateNextState(driveState2);
		driveState2.associateNextState(flipperDownState);
		flipperDownState.associateNextState(liftUpState);
		liftUpState.associateNextState(idleState);
						
		// add states to the network list
		autoNet.addState(driveState);
		autoNet.addState(turnRightState);
		autoNet.addState(driveState2);
		autoNet.addState(flipperDownState);
		autoNet.addState(liftUpState);
		autoNet.addState(idleState);
				
		return autoNet;
	}

	// **** DEPOSIT CUBE SWITCH CENTER LEFT SIDE Network ***** 
	// 1) drive forward for a number of sec
	// 2) Turn LEFT a number of degrees
	// 3) drive forward
	// 4) Turn RIGHT a number of degrees
	// 5) flipper down
	// 6) raise lift
	// 7) drive forward
	// 8) deposit cube
	// 9) go back to idle and stay there 
	private static AutoNetwork createDepositCubeSwitchCenterLeft() {
		
		AutoNetwork autoNet = new AutoNetwork("<Deposit Cube Switch (center left) Network>");
				
		// create states
		AutoState driveState = createMagicDriveState("<Drive State 1>", 18.0, CLOSED_LOOP_VEL_FAST, CLOSED_LOOP_ACCEL_FAST);
		AutoState turnLeftState = createMagicTurnState("<Turn Left State>", -45.0, 1.0, 0.6);
		AutoState driveState2 = createMagicDriveState("<Drive State 2>", 78.0, CLOSED_LOOP_VEL_FAST, CLOSED_LOOP_ACCEL_FAST);
		AutoState turnRightState = createMagicTurnState("<Turn Right State>", 45.0, 1.0, 0.6);
		AutoState flipperDownState = createFlipperState("<Flipper Down State>", false);
		AutoState liftUpState = createLiftState("<Lift Up State>", CubeManagement.SWITCH_LEVEL);
		AutoState driveState3 = createMagicDriveState("<Drive State 3>", 38.0, CLOSED_LOOP_VEL_SLOW, CLOSED_LOOP_ACCEL_SLOW);
		AutoState depositCubeState = createCubeDepositState("<Cube Deposit State>", 3.0);
		AutoState idleState = createIdleState("<Idle State>");

		// connect the state sequence
		driveState.associateNextState(turnLeftState);
		turnLeftState.associateNextState(driveState2);
		driveState2.associateNextState(turnRightState);
		turnRightState.associateNextState(flipperDownState);
		flipperDownState.associateNextState(liftUpState);
		liftUpState.associateNextState(driveState3);
		driveState3.associateNextState(depositCubeState);
		depositCubeState.associateNextState(idleState);
						
		// add states to the network list
		autoNet.addState(driveState);
		autoNet.addState(turnLeftState);
		autoNet.addState(driveState2);
		autoNet.addState(turnRightState);
		autoNet.addState(flipperDownState);
		autoNet.addState(liftUpState);
		autoNet.addState(driveState3);
		autoNet.addState(depositCubeState);
		autoNet.addState(idleState);
				
		return autoNet;
	}

	// **** DEPOSIT CUBE SWITCH CENTER RIGHT SIDE Network ***** 
	// 1) drive forward for a number of sec
	// 2) Turn RIGHT a number of degrees
	// 3) drive forward
	// 4) Turn LEFT a number of degrees
	// 5) flipper down
	// 6) raise lift
	// 7) drive forward
	// 8) deposit cube
	// 9) go back to idle and stay there 
	private static AutoNetwork createDepositCubeSwitchCenterRight() {
		
		AutoNetwork autoNet = new AutoNetwork("<Deposit Cube Switch (center right) Network>");
				
		// create states
		AutoState driveState = createMagicDriveState("<Drive State 1>", 18.0, CLOSED_LOOP_VEL_FAST, CLOSED_LOOP_ACCEL_FAST);
		AutoState turnRightState = createMagicTurnState("<Turn Right State>", 45.0, 1.0, 0.6);
		AutoState driveState2 = createMagicDriveState("<Drive State 2>", 59.0, CLOSED_LOOP_VEL_FAST, CLOSED_LOOP_ACCEL_FAST);
		AutoState turnLeftState = createMagicTurnState("<Turn Left State>", -45.0, 1.0, 0.6);
		AutoState flipperDownState = createFlipperState("<Flipper Down State>", false);
		AutoState liftUpState = createLiftState("<Lift Up State>", CubeManagement.SWITCH_LEVEL);
		AutoState driveState3 = createMagicDriveState("<Drive State 3>", 38.0, CLOSED_LOOP_VEL_SLOW, CLOSED_LOOP_ACCEL_SLOW);
		AutoState depositCubeState = createCubeDepositState("<Cube Deposit State>", 3.0);
		AutoState idleState = createIdleState("<Idle State>");
			
		// connect the state sequence
		driveState.associateNextState(turnRightState);
		turnRightState.associateNextState(driveState2);
		driveState2.associateNextState(turnLeftState);
		turnLeftState.associateNextState(flipperDownState);
		flipperDownState.associateNextState(liftUpState);
		liftUpState.associateNextState(driveState3);
		driveState3.associateNextState(depositCubeState);
		depositCubeState.associateNextState(idleState);
						
		// add states to the network list
		autoNet.addState(driveState);
		autoNet.addState(turnRightState);
		autoNet.addState(driveState2);
		autoNet.addState(turnLeftState);
		autoNet.addState(flipperDownState);
		autoNet.addState(liftUpState);
		autoNet.addState(driveState3);
		autoNet.addState(depositCubeState);
		autoNet.addState(idleState);
				
		return autoNet;
	}

	
	// **** DEPOSIT CUBE SWITCH RIGHT SIDE Network ***** 
	// 1) drive forward for a number of sec
	// 2) Turn LEFT a number of degrees
	// 3) flipper down
	// 4) raise lift
	// 5) drive forward
	// 6) deposit cube
	// 7) go back to idle and stay there 
	private static AutoNetwork createDepositCubeSwitchRight() {
		
		AutoNetwork autoNet = new AutoNetwork("<Deposit Cube Switch (Right Side) Network>");
		
		// create states
		AutoState driveState = createMagicDriveState("<Drive State 1>", 144.0, CLOSED_LOOP_VEL_FAST, CLOSED_LOOP_ACCEL_FAST);
		AutoState turnLeftState = createMagicTurnState("<Turn Left State>", -90.0, 1.0, 0.6);
		AutoState flipperDownState = createFlipperState("<Flipper Down State>", false);
		AutoState liftUpState = createLiftState("<Lift Up State>", CubeManagement.SWITCH_LEVEL);
		AutoState driveState2 = createMagicDriveState("<Drive State 2>", 36.0, CLOSED_LOOP_VEL_SLOW, CLOSED_LOOP_ACCEL_SLOW);
		AutoState depositCubeState = createCubeDepositState("<Deposit Cube State>", 3.0);
		AutoState idleState = createIdleState("<Idle State>");

		// connect the state sequence
		driveState.associateNextState(turnLeftState);
		turnLeftState.associateNextState(flipperDownState);
		flipperDownState.associateNextState(liftUpState);
		liftUpState.associateNextState(driveState2);
		driveState2.associateNextState(depositCubeState);
		depositCubeState.associateNextState(idleState);
						
		// add states to the network list
		autoNet.addState(driveState);
		autoNet.addState(turnLeftState);
		autoNet.addState(flipperDownState);
		autoNet.addState(liftUpState);
		autoNet.addState(driveState2);
		autoNet.addState(depositCubeState);
		autoNet.addState(idleState);
		
		return autoNet;
	}

	// **** DEPOSIT CUBE SCALE RIGHT SIDE Network ***** 
	// 1) drive forward for a number of sec
	// 2) Turn LEFT a number of degrees
	// 3) flipper down
	// 4) Raise lift
	// 5) drive forward
	// 6) Deposit cube
	// 7) go back to idle and stay there 
	private static AutoNetwork createDepositCubeScaleRight() {
		
		AutoNetwork autoNet = new AutoNetwork("<Deposit Cube Scale (Right Side) Network>");
					
		// create states
		AutoState driveState = createMagicDriveState("<Drive State 1>", 302.0, CLOSED_LOOP_VEL_FAST, CLOSED_LOOP_ACCEL_FAST);
		AutoState turnLeftState = createMagicTurnState("<Turn Left State>", -90.0, 1.0, 0.6);
		AutoState flipperDownState = createFlipperState("<Flipper Down State>", false);
		AutoState liftUpState = createLiftState("<Lift Up State>", CubeManagement.SCALE_LEVEL);
		AutoState driveState2 = createMagicDriveState("<Drive State 2>", 24.0, CLOSED_LOOP_VEL_SLOW, CLOSED_LOOP_ACCEL_SLOW);
		AutoState depositCubeState = createCubeDepositState("<Deposit Cube State>", 3.0);
		AutoState idleState = createIdleState("<Idle State>");

		// connect the state sequence
		driveState.associateNextState(turnLeftState);
		turnLeftState.associateNextState(flipperDownState);
		flipperDownState.associateNextState(liftUpState);
		liftUpState.associateNextState(driveState2);
		driveState2.associateNextState(depositCubeState);
		depositCubeState.associateNextState(idleState);
						
		// add states to the network list
		autoNet.addState(driveState);
		autoNet.addState(turnLeftState);
		autoNet.addState(flipperDownState);
		autoNet.addState(liftUpState);
		autoNet.addState(driveState2);
		autoNet.addState(depositCubeState);
		autoNet.addState(idleState);
		
		return autoNet;
	}

	// **** DEPOSIT CUBE SCALE LEFT FROM RIGHT SIDE Network ***** 
	// 1) drive forward for a number of sec
	// 2) Turn LEFT a number of degrees
	// 3) drive forward for a number of sec
	// 4) Turn RIGHT a number of degrees
	// 5) flipper down
	// 6) raise lift
	// 7) drive forward
	// 8) deposit cube
	// 9) go back to idle and stay there 
	private static AutoNetwork createDepositCubeScaleLeftFromRight() {
		
		AutoNetwork autoNet = new AutoNetwork("<Deposit Cube Scale (left from right side) Network>");
		
		// create states
		AutoState driveState = createMagicDriveState("<Drive State 1>", 200.0, CLOSED_LOOP_VEL_FAST, CLOSED_LOOP_ACCEL_FAST);
		AutoState turnLeftState = createMagicTurnState("<Turn Left State>", -90.0, 1.0, 0.6);
		AutoState driveState2 = createMagicDriveState("<Drive State 2>", 256.0, CLOSED_LOOP_VEL_FAST, CLOSED_LOOP_ACCEL_FAST);
		AutoState turnRightState = createMagicTurnState("<Turn Right State>", 135.0, 1.0, 0.6);
		AutoState flipperDownState = createFlipperState("<Flipper Down State>", false);
		AutoState liftUpState = createLiftState("<Lift Up State>", CubeManagement.SCALE_LEVEL);
		AutoState driveState3 = createMagicDriveState("<Drive State 3>", 36.0, CLOSED_LOOP_VEL_SLOW, CLOSED_LOOP_ACCEL_SLOW);
		AutoState depositCubeState = createCubeDepositState("<Deposit Cube State>", 3.0);
		AutoState idleState = createIdleState("<Idle State>");
			
		// connect the state sequence
		driveState.associateNextState(turnLeftState);
		turnLeftState.associateNextState(driveState2);
		driveState2.associateNextState(turnRightState);
		turnRightState.associateNextState(flipperDownState);
		flipperDownState.associateNextState(liftUpState);
		liftUpState.associateNextState(driveState3);
		driveState3.associateNextState(depositCubeState);
		depositCubeState.associateNextState(idleState);
						
		// add states to the network list
		autoNet.addState(driveState);
		autoNet.addState(turnLeftState);
		autoNet.addState(driveState2);
		autoNet.addState(turnRightState);
		autoNet.addState(flipperDownState);
		autoNet.addState(liftUpState);
		autoNet.addState(driveState3);
		autoNet.addState(depositCubeState);
		autoNet.addState(idleState);
				
		return autoNet;
	}	
	
	// **** MOVE TO SCALE LEFT FROM RIGHT SIDE Network ***** 
	// 1) drive forward for a number of sec
	// 2) Turn LEFT a number of degrees
	// 3) drive forward
	// 4) flipper down
	// 5) lift up
	// 6) go back to idle and stay there 
	private static AutoNetwork createMoveToScaleLeftFromRight() {
		
		AutoNetwork autoNet = new AutoNetwork("<Move to Scale Left (Right Side) Network>");
					
		// create states
		AutoState driveState = createMagicDriveState("<Drive State 1>", 200.0, CLOSED_LOOP_VEL_FAST, CLOSED_LOOP_ACCEL_FAST);
		AutoState turnLeftState = createMagicTurnState("<Turn Left State>", -90.0, 1.0, 0.6);
		AutoState driveState2 = createMagicDriveState("<Drive State 2>", 192.0, CLOSED_LOOP_VEL_FAST, CLOSED_LOOP_ACCEL_FAST);
		AutoState flipperDownState = createFlipperState("<Flipper Down State>", false);
		AutoState liftUpState = createLiftState("<Lift Up State>", CubeManagement.SCALE_LEVEL);
		AutoState idleState = createIdleState("<Idle State>");
			
		// connect the state sequence
		driveState.associateNextState(turnLeftState);
		turnLeftState.associateNextState(driveState2);
		driveState2.associateNextState(flipperDownState);
		flipperDownState.associateNextState(liftUpState);
		liftUpState.associateNextState(idleState);
						
		// add states to the network list
		autoNet.addState(driveState);
		autoNet.addState(turnLeftState);
		autoNet.addState(driveState2);
		autoNet.addState(flipperDownState);
		autoNet.addState(liftUpState);
		autoNet.addState(idleState);
		
		return autoNet;
	}
	
	
	/*****************************************************************************************************/
	/**** DEBUG NETWORKS **** Networks below this are used only for debug - disable during competition ***/
	/*****************************************************************************************************/	

	// **** Turning Network ***** 
	// 1) Turn RIGHT 90 degrees a number of times
	// 2) Turn LEFT 90 degrees a number of times
	// 3) go back to step 1 
	private static AutoNetwork createTurningForeverNetwork() {
		
		AutoNetwork autoNet = new AutoNetwork("<Turning Forever Network>");
				
		// create states
		AutoState turnState0 = createMagicTurnState("<Turn 0 State>", -90.0, 1.0, 0.6);
		AutoState turnState1 = createMagicTurnState("<Turn 1 State>", 90.0, 1.0, 0.6);
		AutoState turnState2 = createMagicTurnState("<Turn 2 State>", -90.0, 1.0, 0.6);
		AutoState turnState3 = createMagicTurnState("<Turn 3 State>", 90.0, 1.0, 0.6);
		AutoState turnState4 = createMagicTurnState("<Turn 4 State>", -90.0, 1.0, 0.6);
		AutoState turnState5 = createMagicTurnState("<Turn 5 State>", 90.0, 1.0, 0.6);
		AutoState turnState6 = createMagicTurnState("<Turn 6 State>", -90.0, 1.0, 0.6);
		AutoState turnState7 = createMagicTurnState("<Turn 7 State>", 90.0, 1.0, 0.6);
		AutoState turnState8 = createMagicTurnState("<Turn 8 State>", -90.0, 1.0, 0.6);
		
		// connect the state sequence
		turnState0.associateNextState(turnState1);
		turnState1.associateNextState(turnState2);
		turnState2.associateNextState(turnState3);
		turnState3.associateNextState(turnState4);
		turnState4.associateNextState(turnState5);
		turnState5.associateNextState(turnState6);
		turnState6.associateNextState(turnState7);
		turnState7.associateNextState(turnState8);
		turnState8.associateNextState(turnState0);   // go back to right turning
						
		// add states to the network list
		autoNet.addState(turnState0);
		autoNet.addState(turnState1);
		autoNet.addState(turnState2);
		autoNet.addState(turnState3);
		autoNet.addState(turnState4);
		autoNet.addState(turnState5);
		autoNet.addState(turnState6);
		autoNet.addState(turnState7);
		autoNet.addState(turnState8);
				
		return autoNet;
	}
	
	// **** Turning ONCE Network ***** 
	// 1) Turn RIGHT 90 degrees once
	// 2) go to idle and stay there
	private static AutoNetwork createTurningOnceNetwork() {
		
		AutoNetwork autoNet = new AutoNetwork("<Turning ONCE Network>");
		
		// create the states
		AutoState turnState1 = createMagicTurnState("<Turn 1 State>", 90.0, 1.0, 0.6);
		AutoState idleState = createIdleState("Idle State");
		
		// connect the state sequence
		turnState1.associateNextState(idleState);
						
		// add states to the network list
		autoNet.addState(turnState1);
		autoNet.addState(idleState);
				
		return autoNet;
	}

	// **** Pacing Forever Network - Pace back and forth forever ***** 
	// This network uses absolute headings, and does NOT reset the gyro!
	//
	// 1) be idle for a number of sec
	// 2) drive forward for a number of sec
	// 3) Turn to 180 deg heading
	// 4) drive forward for a number of sec
	// 5) Turn to 0 deg heading
	// 6) Go back to state 2
	private static AutoNetwork createPacingForeverNetwork() {
		
		AutoNetwork autoNet = new AutoNetwork("<Pacing Forever Network>");
				
		// create states
		AutoState driveState1 = createMagicDriveState("<Drive State 1>", 60.0, CLOSED_LOOP_VEL_FAST, CLOSED_LOOP_ACCEL_FAST);
		AutoState turnRightState0 = createMagicTurnState("<Turn Right State 0>", 90.0, 1.0, 0.6);
		AutoState turnRightState1 = createMagicTurnState("<Turn Right State 1>", 90.0, 1.0, 0.6);
		AutoState driveState2 = createMagicDriveState("<Drive State 2>", 60.0, CLOSED_LOOP_VEL_FAST, CLOSED_LOOP_ACCEL_FAST);
		AutoState turnLeftState0 = createMagicTurnState("<Turn Left State 0>", -90.0, 1.0, 0.6);
		AutoState turnLeftState1 = createMagicTurnState("<Turn Left State 1>", -90.0, 1.0, 0.6);
		
		// connect the state sequence
		driveState1.associateNextState(turnRightState0);
		turnRightState0.associateNextState(turnRightState1);
		turnRightState1.associateNextState(driveState2);
		driveState2.associateNextState(turnLeftState0);
		turnLeftState0.associateNextState(turnLeftState1);
		turnLeftState1.associateNextState(driveState1);
						
		// add states to the network list
		autoNet.addState(driveState1);
		autoNet.addState(turnRightState0);
		autoNet.addState(turnRightState1);
		autoNet.addState(driveState2);
		autoNet.addState(turnLeftState0);
		autoNet.addState(turnLeftState1);
				
		return autoNet;
	}

	// **** Lifting Forever Network - Lift up to different levels forever ***** 
	//
	// 1) be idle for a number of sec
	// 2) lift to SWITCH level for a number of sec
	// 3) lift to SCALE level for a number of sec
	// 4) return to SWITCH level for a number of sec
	// 5) return to BASE level for a number of sec
	// 6) Go back to state 2
	private static AutoNetwork createLiftingForeverNetwork() {
		
		AutoNetwork autoNet = new AutoNetwork("<Lifting Forever Network>");
		
		// create states
		AutoState liftState1 = createLiftState("<Lift 1 State>", CubeManagement.liftLevelPulses[CubeManagement.SWITCH_LEVEL]);
		AutoState liftState2 = createLiftState("<Lift 2 State>", CubeManagement.liftLevelPulses[CubeManagement.SCALE_LEVEL]);
		AutoState liftState3 = createLiftState("<Lift 3 State>", CubeManagement.liftLevelPulses[CubeManagement.SWITCH_LEVEL]);
		AutoState liftState4 = createLiftState("<Lift 4 State>", CubeManagement.liftLevelPulses[CubeManagement.BASE_LEVEL]);
		
		// connect the state sequence
		liftState1.associateNextState(liftState2);
		liftState2.associateNextState(liftState3);
		liftState3.associateNextState(liftState4);
		liftState4.associateNextState(liftState1);
						
		// add states to the network list
		autoNet.addState(liftState1);
		autoNet.addState(liftState2);
		autoNet.addState(liftState3);
		autoNet.addState(liftState4);
				
		return autoNet;
	}
	

	
	// **** Test Network - does nothing except transitions states ***** 
	private static AutoNetwork createTestNetwork() {
		
		AutoNetwork autoNet = new AutoNetwork("<Test Network>");
		
		AutoState idleState = new AutoState("<Idle State 1>");
		IdleAction startIdle = new IdleAction("<Start Idle Action 1>");
		IdleAction doSomething2 = new IdleAction("<Placeholder Action 2>");
		IdleAction doSomething3 = new IdleAction("<Placeholder Action 3>");
		TimeEvent timer1 = new TimeEvent(10.0);  // timer event
		idleState.addAction(startIdle);
		idleState.addAction(doSomething2);
		idleState.addAction(doSomething3);
		idleState.addEvent(timer1);
		
		AutoState idleState2 = new AutoState("<Idle State 2>");
		IdleAction startIdle2 = new IdleAction("<Start Idle Action 2>");
		IdleAction doSomething4 = new IdleAction("<Placeholder Action 4>");
		IdleAction doSomething5 = new IdleAction("<Placeholder Action 5>");
		TimeEvent timer2 = new TimeEvent(10.0);  // timer event
		idleState2.addAction(startIdle2);
		idleState2.addAction(doSomething4);
		idleState2.addAction(doSomething5);
		idleState2.addEvent(timer2);
		
		AutoState idleState3 = new AutoState("<Idle State 3>");
		IdleAction startIdle3 = new IdleAction("<Start Idle Action 3>");
		IdleAction doSomething6 = new IdleAction("<Placeholder Action 6>");
		IdleAction doSomething7 = new IdleAction("<Placeholder Action 7>");
		TimeEvent timer3 = new TimeEvent(10.0);  // timer event
		idleState3.addAction(startIdle3);
		idleState3.addAction(doSomething6);
		idleState3.addAction(doSomething7);
		idleState3.addEvent(timer3);
		
		AutoState idleState4 = new AutoState("<Idle State 4>");
		IdleAction deadEnd = new IdleAction("<Dead End Action>");
		idleState4.addAction(deadEnd);
				
		// connect each event with a state to move to
		idleState.associateNextState(idleState2);
		idleState2.associateNextState(idleState3);
		idleState3.associateNextState(idleState4);
						
		autoNet.addState(idleState);
		autoNet.addState(idleState2);
		autoNet.addState(idleState3);
		autoNet.addState(idleState4);
				
		return autoNet;
	}
		
}
