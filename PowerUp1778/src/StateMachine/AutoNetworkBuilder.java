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
	public final static int LIFT_ONCE = 12;
	public final static int TURN_FOREVER = 13;
	public final static int PACE_FOREVER = 14;
	public final static int TURN_ONCE = 15;
	public final static int LIFT_TURN_ONCE = 16;

	// closed-loop position cruise velocity and acceleration (used for all closed-loop position control)
	// units are RPM
	
	// ~3 ft/s - FAST
	private final static int CLOSED_LOOP_VEL_FAST = 900;
	private final static int CLOSED_LOOP_ACCEL_FAST = 300;

	// ~2 ft/s - SLOW
	//private final static int CLOSED_LOOP_VEL_SLOW = 600;     // regionals - values work (but slow)
	//private final static int CLOSED_LOOP_ACCEL_SLOW = 300;
		
	private final static int CLOSED_LOOP_VEL_SLOW = 800; 
	private final static int CLOSED_LOOP_ACCEL_SLOW = 300;

	// ~1 ft/s - VERY SLOW
	//private final static int CLOSED_LOOP_VEL_VERY_SLOW = 300;   
	//private final static int CLOSED_LOOP_ACCEL_VERY_SLOW = 150;

	private final static int CLOSED_LOOP_VEL_VERY_SLOW = 400; 
	private final static int CLOSED_LOOP_ACCEL_VERY_SLOW = 200;
	
	
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

		autoNets.add(LIFT_ONCE, createLiftingOnceNetwork());	
		autoNets.add(TURN_FOREVER, createTurningForeverNetwork());	
		autoNets.add(PACE_FOREVER, createPacingForeverNetwork());	
		autoNets.add(TURN_ONCE, createTurningOnceNetwork());	
		autoNets.add(LIFT_TURN_ONCE, createLiftingTurningOnceNetwork());	

		return autoNets;
	}
	
	///////////////////////////////////////////////////////////
	/*            AutoState Creation Methods                 */
	/*              Single Action States                     */
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
	
	private static AutoState createTimerState(String state_name, double timer_sec)
	{
		AutoState timerState = new AutoState(state_name);
		IdleAction deadEnd = new IdleAction("<Dead End Action>");
		DriveForwardAction driveForwardReset = new DriveForwardAction("<Drive Forward Action -reset>", 0.0, true, 0.0);  // reset gyro
		TimeEvent timer = new TimeEvent(timer_sec);
		timerState.addAction(deadEnd);
		timerState.addAction(driveForwardReset);
		timerState.addEvent(timer);
		
		return timerState;
	}
		
	private static AutoState createMagicDriveState(String state_name, double dist_inches, double error_inches, int max_vel_rpm, int max_accel_rpm, double collector_strength) 
	{		
		AutoState driveState = new AutoState(state_name);
		DriveForwardMagicAction driveForwardMagicAction = new DriveForwardMagicAction("<Drive Forward Magic Action>", dist_inches, max_vel_rpm, max_accel_rpm, true, 0.0);
		CollectCubeAction collectCube = new CollectCubeAction("<Collect Cube Action>",collector_strength);
		//TimeEvent timer = new TimeEvent(2.5);  // drive forward timer event - allow PID time to settle
		ClosedLoopPositionEvent pos = new ClosedLoopPositionEvent(dist_inches, error_inches, 0.6);
		driveState.addAction(driveForwardMagicAction);
		driveState.addAction(collectCube);
		//driveState.addEvent(timer);
		driveState.addEvent(pos);
		
		return driveState;
	}
	
	private static AutoState createCollectorDriveState(String state_name, double dist_inches, double error_inches, int max_vel_rpm, int max_accel_rpm, double collector_strength) 
	{		
		AutoState driveState = new AutoState(state_name);
		DriveForwardMagicAction driveForward = new DriveForwardMagicAction("<Drive Forward Magic Action>", dist_inches, max_vel_rpm, max_accel_rpm, true, 0.0);
		CollectCubeAction collectCube = new CollectCubeAction("<Collect Cube Action>",collector_strength);
		//TimeEvent timer = new TimeEvent(2.5);  // drive forward timer event - allow PID time to settle
		ClosedLoopPositionEvent pos = new ClosedLoopPositionEvent(dist_inches, error_inches, 0.6);
		driveState.addAction(driveForward);
		driveState.addAction(collectCube);
		//driveState.addEvent(timer);
		driveState.addEvent(pos);
		
		return driveState;
	}


	private static AutoState createMagicTurnState(String state_name, double angle_deg, double error_deg, double percent_vbus, double collector_strength)
	{
		AutoState turnState = new AutoState(state_name);
		TurnPIDAction turnPidAction = new TurnPIDAction("<Turn PID action>", angle_deg, percent_vbus, true);
		//TimeEvent timer = new TimeEvent(2.5);  // timer event - allow PID time to settle
		CollectCubeAction collectCube = new CollectCubeAction("<Collect Cube Action>", collector_strength);
		ClosedLoopAngleEvent angle = new ClosedLoopAngleEvent(angle_deg,error_deg, 0.75);
		turnState.addAction(turnPidAction);
		turnState.addAction(collectCube);
		//turnState.addEvent(timer);
		turnState.addEvent(angle);
		
		return turnState;
	}
	
	private static AutoState createLiftState(String state_name, double lift_strength, double lift_timer_sec, double collector_strength)
	{
		AutoState liftState = new AutoState(state_name);
		LiftAction liftAction = new LiftAction("<Lift Action>", lift_strength);
		CollectCubeAction collectCube = new CollectCubeAction("<Collect Cube Action>",collector_strength);
		TimeEvent liftTimer = new TimeEvent(lift_timer_sec); 
		liftState.addAction(liftAction);
		liftState.addAction(collectCube);
		liftState.addEvent(liftTimer);
		
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
	
	///////////////////////////////////////////////////////////
	/*            AutoState Creation Methods                 */
	/*            Multi (combo) Action States                */
	/*            (These are used repeatedly)                */
	///////////////////////////////////////////////////////////
	private static AutoState createLiftAndTurnState(String state_name, double lift_strength, double lift_timer_sec, double angle_deg, double error_deg, double percent_vbus, double collector_strength)
	{
		AutoState liftAndTurnState = new AutoState(state_name);

		CollectCubeAction collectCube = new CollectCubeAction("<Collect Cube Action>",collector_strength);

		LiftAction liftAction = new LiftAction("<Lift Action>", lift_strength);	
		TimeEvent liftTimer = new TimeEvent(lift_timer_sec); 
		
		TurnPIDAction turnPidAction = new TurnPIDAction("<Turn PID action>", angle_deg, percent_vbus, true);
		ClosedLoopAngleEvent angle = new ClosedLoopAngleEvent(angle_deg,error_deg, 0.5);


		liftAndTurnState.addAction(liftAction);
		liftAndTurnState.addAction(turnPidAction);
		liftAndTurnState.addAction(collectCube);
		
		liftAndTurnState.addEvent(liftTimer);
		liftAndTurnState.addEvent(angle);
		liftAndTurnState.setAllEventsTrigger(true);  // BOTH must trigger to leave state
		
		return liftAndTurnState;
	}

	private static AutoState createLiftAndDriveState(String state_name, double lift_strength, double lift_timer_sec, double dist_inches, double error_inches, int max_vel_rpm, int max_accel_rpm, double collector_strength)
	{
		AutoState liftAndDriveState = new AutoState(state_name);

		CollectCubeAction collectCube = new CollectCubeAction("<Collect Cube Action>",collector_strength);

		LiftAction liftAction = new LiftAction("<Lift Action>", lift_strength);	
		TimeEvent liftTimer = new TimeEvent(lift_timer_sec);  // lift timer event
		
		DriveForwardMagicAction driveForwardMagicAction = new DriveForwardMagicAction("<Drive Forward Magic Action>", dist_inches, max_vel_rpm, max_accel_rpm, true, 0.0);
		ClosedLoopPositionEvent pos = new ClosedLoopPositionEvent(dist_inches, error_inches, 0.6);
				
		liftAndDriveState.addAction(liftAction);
		liftAndDriveState.addAction(driveForwardMagicAction);
		liftAndDriveState.addAction(collectCube);
		
		liftAndDriveState.addEvent(liftTimer);
		liftAndDriveState.addEvent(pos);
		liftAndDriveState.setAllEventsTrigger(true);  // BOTH must trigger to leave state
		
		return liftAndDriveState;
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
		AutoState driveState = createMagicDriveState("<Drive State 1>", 120.0, 3.0, CLOSED_LOOP_VEL_SLOW, CLOSED_LOOP_ACCEL_SLOW, CubeManagement.COLLECTOR_IN_AUTO_STRENGTH);
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
	// 2) Turn RIGHT a number of degrees AND raise lift (combo)
	// 3) drive forward
	// 4) deposit cube
	// 5) go back to idle and stay there 
	private static AutoNetwork createDepositCubeSwitchLeft() {
		
		AutoNetwork autoNet = new AutoNetwork("<Deposit Cube Switch (left side) Network>");
			
		// create states
		AutoState driveState = createMagicDriveState("<Drive State 1>", 144.0, 3.0, CLOSED_LOOP_VEL_SLOW, CLOSED_LOOP_ACCEL_SLOW, CubeManagement.COLLECTOR_IN_AUTO_STRENGTH);
		AutoState liftUpTurnRightState = createLiftAndTurnState("<Lift Up & Turn Right State>", -0.7, 1.5 , 90.0, 10.0, 0.35, CubeManagement.COLLECTOR_IN_AUTO_STRENGTH);
		AutoState driveState2 = createMagicDriveState("<Drive State 2>", 11.0, 3.0, CLOSED_LOOP_VEL_SLOW, CLOSED_LOOP_ACCEL_SLOW, CubeManagement.COLLECTOR_IN_AUTO_STRENGTH);
		AutoState depositCubeState = createCubeDepositState("<Deposit Cube State>", 3.0);
		AutoState idleState = createIdleState("<Idle State>");
		
		// connect the state sequence
		driveState.associateNextState(liftUpTurnRightState);
		liftUpTurnRightState.associateNextState(driveState2);
		driveState2.associateNextState(depositCubeState);
		depositCubeState.associateNextState(idleState);
				
		// add states to the network list
		autoNet.addState(driveState);
		autoNet.addState(liftUpTurnRightState);
		autoNet.addState(driveState2);
		autoNet.addState(depositCubeState);
		autoNet.addState(idleState);
				
		return autoNet;
	}
	
	// **** DEPOSIT CUBE SCALE LEFT SIDE Network ***** 
	// 1) drive forward
	// 2) Turn RIGHT a number of degrees AND raise lift (combo)
	// 3) deposit cube
	// 4) go back to idle and stay there 
	private static AutoNetwork createDepositCubeScaleLeft() {
		
		AutoNetwork autoNet = new AutoNetwork("<Deposit Cube Scale (left side) Network>");
		
		// create states
		AutoState driveState = createMagicDriveState("<Drive State 1>", 258.0, 3.0, CLOSED_LOOP_VEL_SLOW, CLOSED_LOOP_ACCEL_SLOW, CubeManagement.COLLECTOR_IN_AUTO_STRENGTH);
		AutoState liftUpTurnRightState = createLiftAndTurnState("<Lift Up Turn Right State>", -0.7, 2.85 , 55.0, 10.0, 0.35, CubeManagement.COLLECTOR_IN_AUTO_STRENGTH);
		AutoState depositCubeState = createCubeDepositState("<Deposit Cube State>", 3.0);
		AutoState idleState = createIdleState("<Idle State>");
			
		// connect the state sequence
		driveState.associateNextState(liftUpTurnRightState);
		liftUpTurnRightState.associateNextState(depositCubeState);
		depositCubeState.associateNextState(idleState);
						
		// add states to the network list
		autoNet.addState(driveState);
		autoNet.addState(liftUpTurnRightState);
		autoNet.addState(depositCubeState);
		autoNet.addState(idleState);
				
		return autoNet;
	}
	
	// **** DEPOSIT CUBE SCALE RIGHT FROM LEFT SIDE Network ***** 
	// 1) drive forward for a number of sec
	// 2) Turn RIGHT a number of degrees
	// 3) drive forward for a number of sec
	// 4) Raise lift AND Turn LEFT a number of degrees (combo)
	// 5) deposit cube
	// 6) go back to idle and stay there 
	private static AutoNetwork createDepositCubeScaleRightFromLeft() {
		
		AutoNetwork autoNet = new AutoNetwork("<Deposit Cube Scale (right from left side) Network>");
				
		// create states
		AutoState driveState = createMagicDriveState("<Drive State 1>", 218.0, 3.0, CLOSED_LOOP_VEL_SLOW, CLOSED_LOOP_ACCEL_SLOW, CubeManagement.COLLECTOR_IN_AUTO_STRENGTH);
		AutoState turnRightState = createMagicTurnState("<Turn Right State>", 90.0, 5.0, 0.3, CubeManagement.COLLECTOR_IN_AUTO_STRENGTH);
		AutoState driveState2 = createMagicDriveState("<Drive State 2>", 234.0, 3.0, CLOSED_LOOP_VEL_SLOW, CLOSED_LOOP_ACCEL_SLOW, CubeManagement.COLLECTOR_IN_AUTO_STRENGTH);
		AutoState turnLeftState = createMagicTurnState("<Turn Left State>", -120.0, 10.0, 0.3, CubeManagement.COLLECTOR_IN_AUTO_STRENGTH);
		AutoState liftAndDriveState = createLiftAndDriveState("<Lift and Drive State>",-0.7, 2.85, 32.0, 3.0, CLOSED_LOOP_VEL_VERY_SLOW, CLOSED_LOOP_ACCEL_VERY_SLOW, CubeManagement.COLLECTOR_IN_AUTO_STRENGTH);
		AutoState depositCubeState = createCubeDepositState("<Deposit Cube State>", 3.0);
		AutoState idleState = createIdleState("<Idle State>");

		// connect the state sequence
		driveState.associateNextState(turnRightState);
		turnRightState.associateNextState(driveState2);
		driveState2.associateNextState(turnLeftState);
		turnLeftState.associateNextState(liftAndDriveState);
		liftAndDriveState.associateNextState(depositCubeState);
		depositCubeState.associateNextState(idleState);
						
		// add states to the network list
		autoNet.addState(driveState);
		autoNet.addState(turnRightState);
		autoNet.addState(driveState2);
		autoNet.addState(turnLeftState);
		autoNet.addState(liftAndDriveState);
		autoNet.addState(depositCubeState);
		autoNet.addState(idleState);
				
		return autoNet;
	}

	// **** MOVE TO SCALE RIGHT FROM LEFT SIDE Network ***** 
	// 1) drive forward for a number of sec
	// 2) Turn RIGHT a number of degrees
	// 3) drive forward
	// 4) go back to idle and stay there 
	private static AutoNetwork createMoveToScaleRightFromLeft() {
		
		AutoNetwork autoNet = new AutoNetwork("<Move to Scale Right (left side) Network>");
				
		// create states
		AutoState driveState = createMagicDriveState("<Drive State 1>", 218.0, 3.0, CLOSED_LOOP_VEL_SLOW, CLOSED_LOOP_ACCEL_SLOW, CubeManagement.COLLECTOR_IN_AUTO_STRENGTH);
		AutoState turnRightState = createMagicTurnState("<Turn Right State>", 90.0, 10.0, 0.3, CubeManagement.COLLECTOR_IN_AUTO_STRENGTH);
		AutoState driveState2 = createMagicDriveState("<Drive State 2>", 160.0, 3.0, CLOSED_LOOP_VEL_SLOW, CLOSED_LOOP_ACCEL_SLOW, CubeManagement.COLLECTOR_IN_AUTO_STRENGTH);
		AutoState idleState = createIdleState("<Idle State>");
		
		// connect the state sequence
		driveState.associateNextState(turnRightState);
		turnRightState.associateNextState(driveState2);
		driveState2.associateNextState(idleState);
						
		// add states to the network list
		autoNet.addState(driveState);
		autoNet.addState(turnRightState);
		autoNet.addState(driveState2);
		autoNet.addState(idleState);
				
		return autoNet;
	}
	
	// **** DEPOSIT CUBE SWITCH CENTER LEFT SIDE Network ***** 
	// 1) drive forward for a number of sec
	// 2) Turn LEFT a number of degrees
	// 3) drive forward
	// 4) Turn RIGHT a number of degrees AND raise lift (combo)
	// 5) drive forward
	// 6) deposit cube
	// 7) back up a bit
	// 8) turn toward the cube pile, lowering the lift in process (combo)
	// 9) drive forward into pile, hopefully collecting a cube
	// 10) drive backward same distance
	// 11) turn back toward switch, raising lift in process (combo)
	// 12) drive forward
	// 13) deposit cube
	// 14) go back to idle and stay there 
	private static AutoNetwork createDepositCubeSwitchCenterLeft() {
		
		AutoNetwork autoNet = new AutoNetwork("<Deposit Cube Switch (center left) Network>");
				
		// create states
		AutoState driveState = createMagicDriveState("<Drive State 1>", 30.0, 3.0, CLOSED_LOOP_VEL_SLOW, CLOSED_LOOP_ACCEL_SLOW, CubeManagement.COLLECTOR_IN_AUTO_STRENGTH);
		AutoState turnLeftState = createMagicTurnState("<Turn Left State>", -40.0, 10.0, 0.3, CubeManagement.COLLECTOR_IN_AUTO_STRENGTH);
		AutoState driveState2 = createMagicDriveState("<Drive State 2>", 80.0, 3.0, CLOSED_LOOP_VEL_SLOW, CLOSED_LOOP_ACCEL_SLOW, CubeManagement.COLLECTOR_IN_AUTO_STRENGTH);
		AutoState liftUpAndTurnRightState = createLiftAndTurnState("<Lift Up and Turn Right State>", -0.7, 1.5, 40.0, 10.0, 0.3, CubeManagement.COLLECTOR_IN_AUTO_STRENGTH);
		AutoState driveState3 = createMagicDriveState("<Drive State 3>", 9.0, 3.0, CLOSED_LOOP_VEL_SLOW, CLOSED_LOOP_ACCEL_SLOW, CubeManagement.COLLECTOR_IN_AUTO_STRENGTH);
		AutoState depositCubeState = createCubeDepositState("<Cube Deposit State>", 0.5);
		AutoState driveState4 = createMagicDriveState("<Drive State 4>", -36.0, 3.0, CLOSED_LOOP_VEL_SLOW, CLOSED_LOOP_ACCEL_SLOW, CubeManagement.COLLECTOR_IN_AUTO_STRENGTH);
		AutoState liftDownAndTurnRightState = createLiftAndTurnState("<Lift Down and Turn Right State>", 0.25, 1.5, 55.0, 10.0, 0.3, CubeManagement.COLLECTOR_IN_AUTO_STRENGTH);
		AutoState driveState5 = createMagicDriveState("<Drive State 5>", 34.0, 3.0, CLOSED_LOOP_VEL_VERY_SLOW, CLOSED_LOOP_ACCEL_VERY_SLOW, CubeManagement.COLLECTOR_IN_AUTOCOLLECT_STRENGTH);
		AutoState driveState6 = createMagicDriveState("<Drive State 6>", -36.0, 3.0, CLOSED_LOOP_VEL_SLOW, CLOSED_LOOP_ACCEL_SLOW, CubeManagement.COLLECTOR_IN_AUTO_STRENGTH);
		AutoState liftUpAndTurnLeftState = createLiftAndTurnState("<Lift Up and Turn Left State>", -0.7, 1.5, -65.0, 10.0, 0.3, CubeManagement.COLLECTOR_IN_AUTO_STRENGTH);
		AutoState driveState7 = createMagicDriveState("<Drive State 7>", 38.0, 3.0, CLOSED_LOOP_VEL_SLOW, CLOSED_LOOP_ACCEL_SLOW, CubeManagement.COLLECTOR_IN_AUTO_STRENGTH);
		AutoState depositCubeState2 = createCubeDepositState("<Cube Deposit State 2>", 1.0);	
		AutoState idleState = createIdleState("<Idle State>");

		// connect the state sequence
		driveState.associateNextState(turnLeftState);
		turnLeftState.associateNextState(driveState2);
		driveState2.associateNextState(liftUpAndTurnRightState);
		liftUpAndTurnRightState.associateNextState(driveState3);
		driveState3.associateNextState(depositCubeState);	
		depositCubeState.associateNextState(driveState4);
		driveState4.associateNextState(liftDownAndTurnRightState);
		liftDownAndTurnRightState.associateNextState(driveState5);
		driveState5.associateNextState(driveState6);
		driveState6.associateNextState(liftUpAndTurnLeftState);	
		liftUpAndTurnLeftState.associateNextState(driveState7);
		driveState7.associateNextState(depositCubeState2);
		depositCubeState2.associateNextState(idleState);
						
		// add states to the network list
		autoNet.addState(driveState);
		autoNet.addState(turnLeftState);
		autoNet.addState(driveState2);
		autoNet.addState(liftUpAndTurnRightState);
		autoNet.addState(driveState3);
		autoNet.addState(depositCubeState);
		autoNet.addState(driveState4);
		autoNet.addState(liftDownAndTurnRightState);
		autoNet.addState(driveState5);
		autoNet.addState(driveState6);
		autoNet.addState(liftUpAndTurnLeftState);
		autoNet.addState(driveState7);
		autoNet.addState(depositCubeState2);
		autoNet.addState(idleState);
				
		return autoNet;
	}

	// **** DEPOSIT CUBE SWITCH CENTER RIGHT SIDE Network ***** 
	// 1) drive forward for a number of sec
	// 2) Turn RIGHT a number of degrees
	// 3) drive forward
	// 4) Turn LEFT a number of degrees AND raise lift (combo)
	// 5) drive forward
	// 6) deposit cube
	// 7) back up a bit
	// 8) turn toward the cube pile, lowering the lift in process (combo)
	// 9) drive forward into pile, hopefully collecting a cube
	// 10) drive backward same distance
	// 11) turn back toward switch, raising lift in process (combo)
	// 12) drive forward
	// 13) deposit cube
	// 14) go back to idle and stay there 
	private static AutoNetwork createDepositCubeSwitchCenterRight() {
		
		AutoNetwork autoNet = new AutoNetwork("<Deposit Cube Switch (center right) Network>");
				
		// create states
		AutoState driveState = createMagicDriveState("<Drive State 1>", 30.0, 3.0, CLOSED_LOOP_VEL_SLOW, CLOSED_LOOP_ACCEL_SLOW, CubeManagement.COLLECTOR_IN_AUTO_STRENGTH);
		AutoState turnRightState = createMagicTurnState("<Turn Right State>", 40.0, 10.0, 0.3, CubeManagement.COLLECTOR_IN_AUTO_STRENGTH);
		AutoState driveState2 = createMagicDriveState("<Drive State 2>", 76.0, 3.0, CLOSED_LOOP_VEL_SLOW, CLOSED_LOOP_ACCEL_SLOW, CubeManagement.COLLECTOR_IN_AUTO_STRENGTH);
		AutoState liftUpAndTurnLeftState = createLiftAndTurnState("<Lift Up and Turn Left State>", -0.7, 1.5, -40.0, 10.0, 0.3, CubeManagement.COLLECTOR_IN_AUTO_STRENGTH);
		AutoState driveState3 = createMagicDriveState("<Drive State 3>", 9.0, 3.0, CLOSED_LOOP_VEL_SLOW, CLOSED_LOOP_ACCEL_SLOW, CubeManagement.COLLECTOR_IN_AUTO_STRENGTH);
		AutoState depositCubeState = createCubeDepositState("<Cube Deposit State>", 0.5);
		AutoState driveState4 = createMagicDriveState("<Drive State 4>", -36.0, 3.0, CLOSED_LOOP_VEL_SLOW, CLOSED_LOOP_ACCEL_SLOW, CubeManagement.COLLECTOR_IN_AUTO_STRENGTH);
		AutoState liftDownAndTurnLeftState = createLiftAndTurnState("<Lift Down and Turn Left State>", 0.25, 1.5, -55.0, 10.0, 0.3, CubeManagement.COLLECTOR_IN_AUTO_STRENGTH);
		AutoState driveState5 = createMagicDriveState("<Drive State 5>", 34.0, 3.0, CLOSED_LOOP_VEL_VERY_SLOW, CLOSED_LOOP_ACCEL_VERY_SLOW, CubeManagement.COLLECTOR_IN_AUTOCOLLECT_STRENGTH);
		AutoState driveState6 = createMagicDriveState("<Drive State 6>", -28.0, 3.0, CLOSED_LOOP_VEL_SLOW, CLOSED_LOOP_ACCEL_SLOW, CubeManagement.COLLECTOR_IN_AUTO_STRENGTH);
		AutoState liftUpAndTurnRightState = createLiftAndTurnState("<Lift Up and Turn Right State>", -0.7, 1.5, 55.0, 10.0, 0.3, CubeManagement.COLLECTOR_IN_AUTO_STRENGTH);
		AutoState driveState7 = createMagicDriveState("<Drive State 7>", 38.0, 3.0, CLOSED_LOOP_VEL_SLOW, CLOSED_LOOP_ACCEL_SLOW, CubeManagement.COLLECTOR_IN_AUTO_STRENGTH);
		AutoState depositCubeState2 = createCubeDepositState("<Cube Deposit State 2>", 1.0);	
		AutoState idleState = createIdleState("<Idle State>");

		// connect the state sequence
		driveState.associateNextState(turnRightState);
		turnRightState.associateNextState(driveState2);
		driveState2.associateNextState(liftUpAndTurnLeftState);
		liftUpAndTurnLeftState.associateNextState(driveState3);
		driveState3.associateNextState(depositCubeState);
		depositCubeState.associateNextState(driveState4);		
		driveState4.associateNextState(liftDownAndTurnLeftState);
		liftDownAndTurnLeftState.associateNextState(driveState5);
		driveState5.associateNextState(driveState6);
		driveState6.associateNextState(liftUpAndTurnRightState);	
		liftUpAndTurnRightState.associateNextState(driveState7);
		driveState7.associateNextState(depositCubeState2);
		depositCubeState2.associateNextState(idleState);
						
		// add states to the network list
		autoNet.addState(driveState);
		autoNet.addState(turnRightState);
		autoNet.addState(driveState2);
		autoNet.addState(liftUpAndTurnLeftState);
		autoNet.addState(driveState3);
		autoNet.addState(depositCubeState);
		autoNet.addState(driveState4);
		autoNet.addState(liftDownAndTurnLeftState);
		autoNet.addState(driveState5);
		autoNet.addState(driveState6);
		autoNet.addState(liftUpAndTurnRightState);
		autoNet.addState(driveState7);
		autoNet.addState(depositCubeState2);
		autoNet.addState(idleState);
				
		return autoNet;
	}

	
	// **** DEPOSIT CUBE SWITCH RIGHT SIDE Network ***** 
	// 1) drive forward for a number of sec
	// 2) Raise lift AND Turn LEFT a number of degrees (combo)
	// 3) drive forward
	// 4) deposit cube
	// 5) go back to idle and stay there 
	private static AutoNetwork createDepositCubeSwitchRight() {
		
		AutoNetwork autoNet = new AutoNetwork("<Deposit Cube Switch (Right Side) Network>");
		
		// create states
		AutoState driveState = createMagicDriveState("<Drive State 1>", 144.0, 3.0, CLOSED_LOOP_VEL_SLOW, CLOSED_LOOP_ACCEL_SLOW, CubeManagement.COLLECTOR_IN_AUTO_STRENGTH);
		AutoState liftUpAndTurnLeftState = createLiftAndTurnState("<Lift Up & Turn Left State>", -0.7, 1.5, -90.0, 10.0, 0.3, CubeManagement.COLLECTOR_IN_AUTO_STRENGTH);
		AutoState driveState2 = createMagicDriveState("<Drive State 2>", 11.0, 3.0, CLOSED_LOOP_VEL_SLOW, CLOSED_LOOP_ACCEL_SLOW, CubeManagement.COLLECTOR_IN_AUTO_STRENGTH);
		AutoState depositCubeState = createCubeDepositState("<Deposit Cube State>", 3.0);
		AutoState idleState = createIdleState("<Idle State>");

		// connect the state sequence
		driveState.associateNextState(liftUpAndTurnLeftState);
		liftUpAndTurnLeftState.associateNextState(driveState2);
		driveState2.associateNextState(depositCubeState);
		depositCubeState.associateNextState(idleState);
						
		
		// add states to the network list
		autoNet.addState(driveState);
		autoNet.addState(liftUpAndTurnLeftState);
		autoNet.addState(driveState2);
		autoNet.addState(depositCubeState);
		autoNet.addState(idleState);
		
		return autoNet;
	}
	
	// **** DEPOSIT CUBE SCALE RIGHT SIDE Network ***** 
	// 1) drive forward
	// 2) raise lift AND turn LEFT a number of degrees (combo)
	// 3) deposit cube
	// 4) go back to idle and stay there 
	private static AutoNetwork createDepositCubeScaleRight() {
		
		AutoNetwork autoNet = new AutoNetwork("<Deposit Cube Scale (Right Side) Network>");
					
		// create states
		AutoState driveState = createMagicDriveState("<Drive State 1>", 258.0, 3.0, CLOSED_LOOP_VEL_SLOW, CLOSED_LOOP_ACCEL_SLOW, CubeManagement.COLLECTOR_IN_AUTO_STRENGTH);
		AutoState liftUpTurnLeftState = createLiftAndTurnState("<Lift Up Turn Lefts State>", -0.7, 2.85, -55.0, 10.0, 0.3, CubeManagement.COLLECTOR_IN_AUTO_STRENGTH);
		AutoState depositCubeState = createCubeDepositState("<Deposit Cube State>", 3.0);
		AutoState idleState = createIdleState("<Idle State>");
			
		// connect the state sequence
		driveState.associateNextState(liftUpTurnLeftState);
		liftUpTurnLeftState.associateNextState(depositCubeState);
		depositCubeState.associateNextState(idleState);
						
		// add states to the network list
		autoNet.addState(driveState);
		autoNet.addState(liftUpTurnLeftState);
		autoNet.addState(depositCubeState);
		autoNet.addState(idleState);
		
		return autoNet;
	}

	// **** DEPOSIT CUBE SCALE LEFT FROM RIGHT SIDE Network ***** 
	// 1) drive forward for a number of sec
	// 2) Turn LEFT a number of degrees
	// 3) drive forward for a number of sec
	// 4) Raise lift AND Turn RIGHT a number of degrees (combo)
	// 5) deposit cube
	// 6) go back to idle and stay there 
	private static AutoNetwork createDepositCubeScaleLeftFromRight() {
		
		AutoNetwork autoNet = new AutoNetwork("<Deposit Cube Scale (left from right side) Network>");
		
		// create states
		AutoState driveState = createMagicDriveState("<Drive State 1>", 218.0, 3.0, CLOSED_LOOP_VEL_SLOW, CLOSED_LOOP_ACCEL_SLOW, CubeManagement.COLLECTOR_IN_AUTO_STRENGTH);
		AutoState turnLeftState = createMagicTurnState("<Turn Left State>", -90.0, 5.0, 0.3, CubeManagement.COLLECTOR_IN_AUTO_STRENGTH);
		AutoState driveState2 = createMagicDriveState("<Drive State 2>", 234.0, 3.0, CLOSED_LOOP_VEL_SLOW, CLOSED_LOOP_ACCEL_SLOW, CubeManagement.COLLECTOR_IN_AUTO_STRENGTH);
		AutoState turnRightState = createMagicTurnState("<Turn Right State>", 120.0, 10.0, 0.3, CubeManagement.COLLECTOR_IN_AUTO_STRENGTH);
		AutoState liftAndDriveState = createLiftAndDriveState("<Lift and Drive State>",-0.7, 2.75, 32.0, 3.0, CLOSED_LOOP_VEL_VERY_SLOW, CLOSED_LOOP_ACCEL_VERY_SLOW, CubeManagement.COLLECTOR_IN_AUTO_STRENGTH);
		AutoState depositCubeState = createCubeDepositState("<Deposit Cube State>", 3.0);
		AutoState idleState = createIdleState("<Idle State>");
			
		// connect the state sequence
		driveState.associateNextState(turnLeftState);
		turnLeftState.associateNextState(driveState2);
		driveState2.associateNextState(turnRightState);
		turnRightState.associateNextState(liftAndDriveState);
		liftAndDriveState.associateNextState(depositCubeState);
		depositCubeState.associateNextState(idleState);
						
		// add states to the network list
		autoNet.addState(driveState);
		autoNet.addState(turnLeftState);
		autoNet.addState(driveState2);
		autoNet.addState(turnRightState);
		autoNet.addState(liftAndDriveState);
		autoNet.addState(depositCubeState);
		autoNet.addState(idleState);
				
		return autoNet;
	}	
	
	// **** MOVE TO SCALE LEFT FROM RIGHT SIDE Network ***** 
	// 1) drive forward for a number of sec
	// 2) Turn LEFT a number of degrees
	// 3) drive forward
	// 4) go back to idle and stay there 
	private static AutoNetwork createMoveToScaleLeftFromRight() {
		
		AutoNetwork autoNet = new AutoNetwork("<Move to Scale Left (Right Side) Network>");
							
		// create states
		AutoState driveState = createMagicDriveState("<Drive State 1>", 218.0, 3.0, CLOSED_LOOP_VEL_SLOW, CLOSED_LOOP_ACCEL_SLOW, CubeManagement.COLLECTOR_IN_AUTO_STRENGTH);
		AutoState turnLeftState = createMagicTurnState("<Turn Left State>", -90.0, 10.0, 0.3, CubeManagement.COLLECTOR_IN_AUTO_STRENGTH);
		AutoState driveState2 = createMagicDriveState("<Drive State 2>", 160.0, 3.0, CLOSED_LOOP_VEL_SLOW, CLOSED_LOOP_ACCEL_SLOW, CubeManagement.COLLECTOR_IN_AUTO_STRENGTH);
		AutoState idleState = createIdleState("<Idle State>");
			
		// connect the state sequence
		driveState.associateNextState(turnLeftState);
		turnLeftState.associateNextState(driveState2);
		driveState2.associateNextState(idleState);
						
		// add states to the network list
		autoNet.addState(driveState);
		autoNet.addState(turnLeftState);
		autoNet.addState(driveState2);
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
		AutoState turnState0 = createMagicTurnState("<Turn 0 State>", -90.0, 5.0, 0.35, CubeManagement.COLLECTOR_IN_AUTO_STRENGTH);
		AutoState turnState1 = createMagicTurnState("<Turn 1 State>", 90.0, 5.0, 0.35, CubeManagement.COLLECTOR_IN_AUTO_STRENGTH);
		AutoState turnState2 = createMagicTurnState("<Turn 2 State>", -90.0, 5.0, 0.35, CubeManagement.COLLECTOR_IN_AUTO_STRENGTH);
		AutoState turnState3 = createMagicTurnState("<Turn 3 State>", 90.0, 5.0, 0.35, CubeManagement.COLLECTOR_IN_AUTO_STRENGTH);
		AutoState turnState4 = createMagicTurnState("<Turn 4 State>", -90.0, 5.0, 0.35, CubeManagement.COLLECTOR_IN_AUTO_STRENGTH);
		AutoState turnState5 = createMagicTurnState("<Turn 5 State>", 90.0, 5.0, 0.35, CubeManagement.COLLECTOR_IN_AUTO_STRENGTH);
		AutoState turnState6 = createMagicTurnState("<Turn 6 State>", -90.0, 5.0, 0.35, CubeManagement.COLLECTOR_IN_AUTO_STRENGTH);
		AutoState turnState7 = createMagicTurnState("<Turn 7 State>", 90.0, 5.0, 0.35, CubeManagement.COLLECTOR_IN_AUTO_STRENGTH);
		AutoState turnState8 = createMagicTurnState("<Turn 8 State>", -90.0, 5.0, 0.35, CubeManagement.COLLECTOR_IN_AUTO_STRENGTH);
		
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
		AutoState turnState1 = createMagicTurnState("<Turn 1 State>", 90.0, 5.0, 0.35, CubeManagement.COLLECTOR_IN_AUTO_STRENGTH);
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
		AutoState driveState1 = createMagicDriveState("<Drive State 1>", 60.0, 3.0, CLOSED_LOOP_VEL_SLOW, CLOSED_LOOP_ACCEL_SLOW, CubeManagement.COLLECTOR_IN_AUTO_STRENGTH);
		AutoState turnRightState0 = createMagicTurnState("<Turn Right State 0>", 90.0, 5.0, 0.35, CubeManagement.COLLECTOR_IN_AUTO_STRENGTH);
		AutoState turnRightState1 = createMagicTurnState("<Turn Right State 1>", 90.0, 5.0, 0.35, CubeManagement.COLLECTOR_IN_AUTO_STRENGTH);
		AutoState driveState2 = createMagicDriveState("<Drive State 2>", 60.0, 3.0, CLOSED_LOOP_VEL_SLOW, CLOSED_LOOP_ACCEL_SLOW, CubeManagement.COLLECTOR_IN_AUTO_STRENGTH);
		AutoState turnLeftState0 = createMagicTurnState("<Turn Left State 0>", -90.0, 5.0, 0.35, CubeManagement.COLLECTOR_IN_AUTO_STRENGTH);
		AutoState turnLeftState1 = createMagicTurnState("<Turn Left State 1>", -90.0, 5.0, 0.35, CubeManagement.COLLECTOR_IN_AUTO_STRENGTH);
		
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

	// **** Lifting Once Network ***** 
	//
	// 1) Raise lift for a number of sec
	// 2) deposit cube for a number of sec
	// 3) Lower lift for a number of sec
	// 4) Go to idle
	private static AutoNetwork createLiftingOnceNetwork() {
		
		AutoNetwork autoNet = new AutoNetwork("<Lifting Once Network>");
		
		// create states
		AutoState liftUpState = createLiftState("<Lift Up State>", -0.7, 2.85, CubeManagement.COLLECTOR_IN_AUTO_STRENGTH);
		AutoState depositCubeState = createCubeDepositState("<Deposit Cube State>", 0.5);
		AutoState liftDownState = createLiftState("<Lift Down State>", 0.2, 1.5, CubeManagement.COLLECTOR_IN_AUTO_STRENGTH);
		AutoState idleState = createIdleState("<Idle State>");
		
		// connect the state sequence
		liftUpState.associateNextState(depositCubeState);
		depositCubeState.associateNextState(liftDownState);
		liftDownState.associateNextState(idleState);
						
		// add states to the network list
		autoNet.addState(liftUpState);
		autoNet.addState(depositCubeState);
		autoNet.addState(liftDownState);
		autoNet.addState(idleState);
				
		return autoNet;
	}
	
	// **** Lifting & Turning Once Network - Lift up to specified level, while turning ***** 
	//
	// 1) be idle for a number of sec
	// 2) lift to a level for a number of sec
	// 3) expel cube for a number of sec
	// 4) Go to idle
	private static AutoNetwork createLiftingTurningOnceNetwork() {
		
		AutoNetwork autoNet = new AutoNetwork("<Lifting & Turning Once Network>");
		
		// create states
		AutoState liftandTurnState1 = createLiftAndTurnState("<Lift and Turn State>", -0.7, 1.5, 45.0, 5.0, 0.35, CubeManagement.COLLECTOR_IN_AUTO_STRENGTH);
		AutoState depositCubeState = createCubeDepositState("<Deposit Cube State>", 3.0);
		AutoState idleState = createIdleState("<Idle State>");
		
		// connect the state sequence
		liftandTurnState1.associateNextState(depositCubeState);
		depositCubeState.associateNextState(idleState);
						
		// add states to the network list
		autoNet.addState(liftandTurnState1);
		autoNet.addState(depositCubeState);
		autoNet.addState(idleState);
				
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
