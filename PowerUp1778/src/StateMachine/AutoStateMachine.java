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

	// computes binary value from digital inputs.  
	// If all switches are false (zero), auto is disabled
	private int getNetworkIndex()
	{
		// grab the selected action and position from the driver station
		int action = autoChooser.getAction();
		int position = autoChooser.getPosition();
		
		int netIndex = AutoNetworkBuilder.DO_NOTHING;
		
		if ((action == AutoChooser.DO_NOTHING) || (position == AutoChooser.POS_UNDEFINED))
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
		else  // CUBE OPS, depends on field config and position
		{
			autoNetworkEnable = true;
			
			switch (position) {
			case AutoChooser.LEFT_POSITION:
				netIndex = leftPositionLogic(action);
				break;
			case AutoChooser.CENTER_POSITION:
				netIndex = centerPositionLogic(action);
				break;
			case AutoChooser.RIGHT_POSITION:
				netIndex = rightPositionLogic(action);
				break;
			default:
				// no position defined - do nothing
				autoNetworkEnable = false;
			}
		}
		
		// return index value for network selected
		return netIndex;
		
	}
	
	// logic for left position and desired action
	private int leftPositionLogic(int desiredAction)
	{
		int netIndex = AutoNetworkBuilder.DO_NOTHING;
		
		if (fieldAllianceColors[SWITCH] == LEFT) {
			// first priority - turn on switch
			netIndex = AutoNetworkBuilder.DEPOSIT_CUBE_SWITCH_LEFT;
		}
		else if (fieldAllianceColors[SCALE] == LEFT) {
			// second priority - turn on scale
			netIndex = AutoNetworkBuilder.DEPOSIT_CUBE_SCALE_LEFT;
		}
		else {
			// third priority - move into position for other side of scale
			netIndex = AutoNetworkBuilder.MOVE_TO_SCALE_RIGHT_FROM_LEFT;
		}
		
		return netIndex;
	}
	
	private int centerPositionLogic(int desiredAction)
	{
		int netIndex = AutoNetworkBuilder.DO_NOTHING;
		
		if (fieldAllianceColors[SWITCH] == LEFT) {
			// first priority - turn on switch
			netIndex = AutoNetworkBuilder.DEPOSIT_CUBE_SWITCH_CENTER_LEFT;
		}
		else if (fieldAllianceColors[SWITCH] == RIGHT) {
			// second priority - turn on scale
			netIndex = AutoNetworkBuilder.DEPOSIT_CUBE_SWITCH_CENTER_RIGHT;
		}
		else {
			// this should never happen unless field alliance colors are undefined
			netIndex = AutoNetworkBuilder.DRIVE_FORWARD;
		}

		return netIndex;
	}
	
	private int rightPositionLogic(int desiredAction) 
	{
		int netIndex = AutoNetworkBuilder.DO_NOTHING;
		
		if (fieldAllianceColors[SWITCH] == RIGHT) {
			// first priority - turn on switch
			netIndex = AutoNetworkBuilder.DEPOSIT_CUBE_SWITCH_RIGHT;
		}
		else if (fieldAllianceColors[SCALE] == RIGHT) {
			// second priority - turn on scale
			netIndex = AutoNetworkBuilder.DEPOSIT_CUBE_SCALE_RIGHT;
		}
		else {
			// third priority - move into position for other side of scale
			netIndex = AutoNetworkBuilder.MOVE_TO_SCALE_LEFT_FROM_RIGHT;
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
