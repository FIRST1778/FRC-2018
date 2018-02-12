package StateMachine;

import java.util.ArrayList;
import NetworkComm.InputOutputComm;
import Systems.NavXSensor;
import edu.wpi.first.wpilibj.DriverStation;

public class AutoStateMachine {
		
	private boolean autoNetworkEnable = false;
		
	private ArrayList<AutoNetwork> autoNetworks;

	private final int SWITCH = 0;
	private final int SCALE = 1;
	private final int OPP_SWITCH = 2;
	
	private final int UNDEFINED = 0;
	private final int LEFT = 1;
	private final int RIGHT = 2;
	
	private int[] fieldAllianceColors = {UNDEFINED, UNDEFINED, UNDEFINED};
	
	private AutoNetwork currentNetwork;
	private AutoChooser autoChooser;
		
	public AutoStateMachine()
	{
		AutoNetworkBuilder.initialize();	
		InputOutputComm.initialize();
		NavXSensor.initialize();
		
		// create list of autonomous networks
		autoNetworks = AutoNetworkBuilder.readInNetworks();
		
		// create the smart dashboard chooser
		autoChooser = new AutoChooser();
		
	}
				
	public void start()
	{
				
		// check switch and scale lighting combination
		getFieldColorConfig();
		
		// determine if we are running auto or not
		int networkIndex = getNetworkIndex();
		
		String myString = new String("autoNetworkEnable = " + autoNetworkEnable + ", networkIndex = " + networkIndex);
		System.out.println(myString);
		InputOutputComm.putString(InputOutputComm.LogTable.kMainLog,"Auto/AutoSM_network", myString);
		
		if (autoNetworkEnable)
		{
			// if we have a state network
			currentNetwork = autoNetworks.get(networkIndex);
			
			if (currentNetwork != null)
			{	
				//System.out.println("State machine starting with " + currentState.name);						
				currentNetwork.enter();
			}
		}
		
		// must initialize the gyro class and reset the angle to our initial position
		NavXSensor.reset();
		
	}
	
	public void process()  {
		
		if (autoNetworkEnable)
		{
			// process the current network
			if (currentNetwork != null)
			{
				currentNetwork.process();
			}	
		}
	}
	
	public void stop()  {
		if (currentNetwork != null)
		{
			currentNetwork.exit();
		}	
		
	}

	// figures out which autonomous network to run 
	private int getNetworkIndex()
	{
		// grab the selected action and position from the driver station
		int action = autoChooser.getAction();
		int position = autoChooser.getPosition();
		int right_left_priority = autoChooser.getRightLeftPriority();
		int remote_scale_action = autoChooser.getRemoteScaleAction();
		
		int netIndex = AutoNetworkBuilder.DO_NOTHING;
		
		if (action == AutoChooser.DO_NOTHING)
		{
			// auto state machine operation disabled
			autoNetworkEnable = false;
		}
		else if (action == AutoChooser.DRIVE_FORWARD)
		{
			// override simple drive forward network
			autoNetworkEnable = true;
			netIndex = AutoNetworkBuilder.DRIVE_FORWARD;
		}
		else if (action == AutoChooser.CUBE_OPS) // CUBE OPS, depends on field config and position
		{
			autoNetworkEnable = true;
			
			switch (position) {
			case AutoChooser.LEFT_POSITION:
				netIndex = leftPositionLogic(right_left_priority, remote_scale_action);
				break;
			case AutoChooser.CENTER_POSITION:
				netIndex = centerPositionLogic();
				break;
			case AutoChooser.RIGHT_POSITION:
				netIndex = rightPositionLogic(right_left_priority, remote_scale_action);
				break;
			default:
				// no position defined - do nothing
				autoNetworkEnable = false;
			}
		}
		else if (action == AutoChooser.LIFT_FOREVER)
		{
			// debug network
			autoNetworkEnable = true;
			netIndex = AutoNetworkBuilder.LIFT_FOREVER;
		}
		else if (action == AutoChooser.TURN_FOREVER)
		{
			// debug network
			autoNetworkEnable = true;
			netIndex = AutoNetworkBuilder.TURN_FOREVER;
		}
		else if (action == AutoChooser.PACE_FOREVER)
		{
			// debug network
			autoNetworkEnable = true;
			netIndex = AutoNetworkBuilder.PACE_FOREVER;
		}
		else if (action == AutoChooser.TURN_ONCE)
		{
			// debug network
			autoNetworkEnable = true;
			netIndex = AutoNetworkBuilder.TURN_ONCE;
		}
		else {
			autoNetworkEnable = false;
		}
		
		// return index value for network selected
		return netIndex;
		
	}
	
	// logic for left position
	private int leftPositionLogic(int right_left_priority, int remote_scale_action)
	{
		int netIndex = AutoNetworkBuilder.DO_NOTHING;
		
		// check local scale priority over local switch
		if ((right_left_priority == AutoChooser.SCALE_ONE_CUBE) ||
		   (right_left_priority == AutoChooser.SCALE_TWO_CUBES))
		{
			// check local scale over local switch
			if (fieldAllianceColors[SCALE] == LEFT) 
			{
				if (right_left_priority == AutoChooser.SCALE_ONE_CUBE) {
					// first priority - deposit one cube on scale (same side)
					netIndex = AutoNetworkBuilder.DEPOSIT_CUBE_SCALE_LEFT;
				}
				else {
					// next priority - deposit TWO cubes on scale (same side)
					netIndex = AutoNetworkBuilder.DEPOSIT_CUBE_SCALE_LEFT_TWO_CUBES;
				}
			}
			else // (fieldAllianceColors[SCALE] == RIGHT) 
			{
				// first priority - move on scale (remote side)
				netIndex = remoteScaleAction(LEFT,remote_scale_action);
			}
		}
		else // (right_left_priority == AutoChooser.SWITCH)
		{
			// check local switch over local scale
			if (fieldAllianceColors[SWITCH] == LEFT) 
			{
				// first priority - deposit cube on switch (same side)
				netIndex = AutoNetworkBuilder.DEPOSIT_CUBE_SWITCH_LEFT;
			}
			else if (fieldAllianceColors[SCALE] == LEFT) 
			{
				// second priority - deposit one cube on scale (same side)
				netIndex = AutoNetworkBuilder.DEPOSIT_CUBE_SCALE_LEFT;
			}
			else // (fieldAllianceColors[SCALE] == RIGHT) 
			{
				// first priority - move on scale (remote side)
				netIndex = remoteScaleAction(LEFT,remote_scale_action);
			}
		}
						
		return netIndex;
	}
	
	
	private int centerPositionLogic()
	{
		int netIndex = AutoNetworkBuilder.DO_NOTHING;
		
		if (fieldAllianceColors[SWITCH] == LEFT) {
			// first priority - turn to left side of switch
			netIndex = AutoNetworkBuilder.DEPOSIT_CUBE_SWITCH_CENTER_LEFT;
		}
		else {
			// second priority - turn to right side of switch
			netIndex = AutoNetworkBuilder.DEPOSIT_CUBE_SWITCH_CENTER_RIGHT;
		}

		return netIndex;
	}
	
	// logic for right position
	private int rightPositionLogic(int right_left_priority, int remote_scale_action)
	{
		int netIndex = AutoNetworkBuilder.DO_NOTHING;
		
		// check local scale priority over local switch
		if ((right_left_priority == AutoChooser.SCALE_ONE_CUBE) ||
			(right_left_priority == AutoChooser.SCALE_TWO_CUBES))
		{
			// check local scale over local switch
			if (fieldAllianceColors[SCALE] == RIGHT) 
			{
				if (right_left_priority == AutoChooser.SCALE_ONE_CUBE) {
					// first priority - deposit one cube on scale (same side)
					netIndex = AutoNetworkBuilder.DEPOSIT_CUBE_SCALE_RIGHT;
				}
				else {
					// next priority - deposit TWO cubes on scale (same side)
					netIndex = AutoNetworkBuilder.DEPOSIT_CUBE_SCALE_RIGHT_TWO_CUBES;					
				}
			}
			else // (fieldAllianceColors[SCALE] == LEFT) 
			{
				// first priority - move on scale (remote side)
				netIndex = remoteScaleAction(RIGHT,remote_scale_action);
			}
		}
		else // (right_left_priority == AutoChooser.SWITCH)
		{
			// check local switch over local scale
			if (fieldAllianceColors[SWITCH] == RIGHT) 
			{
				// first priority - deposit cube on switch (same side)
				netIndex = AutoNetworkBuilder.DEPOSIT_CUBE_SWITCH_RIGHT;
			}
			else if (fieldAllianceColors[SCALE] == RIGHT) 
			{
				// second priority - deposit one cube on scale (same side)
				netIndex = AutoNetworkBuilder.DEPOSIT_CUBE_SCALE_RIGHT;
			}
			else // (fieldAllianceColors[SCALE] == LEFT) 
			{
				// first priority - move on scale (remote side)
				netIndex = remoteScaleAction(RIGHT,remote_scale_action);
			}
		}
						
		return netIndex;
	}
	
	private int remoteScaleAction(int fromPos, int remote_scale_action)
	{
		int netIndex;
		
		if (fromPos == LEFT) {
			if (remote_scale_action == AutoChooser.REMOTE_SCALE_CUBE_DROP) 
			{
				// move to other side of scale and deposit cube
				netIndex = AutoNetworkBuilder.DEPOSIT_CUBE_SCALE_RIGHT_FROM_LEFT;
			}
			else
			{
				// move into position for other side of scale
				netIndex = AutoNetworkBuilder.MOVE_TO_SCALE_RIGHT_FROM_LEFT;
			}
		}
		else  // (fromPos == RIGHT)
		{
			if (remote_scale_action == AutoChooser.REMOTE_SCALE_CUBE_DROP) 
			{
				// move to other side of scale and deposit cube
				netIndex = AutoNetworkBuilder.DEPOSIT_CUBE_SCALE_LEFT_FROM_RIGHT;
			}
			else
			{
				// move into position for other side of scale
				netIndex = AutoNetworkBuilder.MOVE_TO_SCALE_LEFT_FROM_RIGHT;
			}
			
		}
		
		return netIndex;
	}
	
	// retrieves color configuration of field elements relative to alliance side
	private void getFieldColorConfig()
	{
		String gameData;
		gameData = DriverStation.getInstance().getGameSpecificMessage();

		for (int i=0; i < 3; i++)
		{
			if (gameData.charAt(i) == 'L')
				fieldAllianceColors[i] = LEFT;
			else
				fieldAllianceColors[i] = RIGHT;
		}
		InputOutputComm.putString(InputOutputComm.LogTable.kMainLog,"Auto/FieldConfig", gameData);

	}
	
}
