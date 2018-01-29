package StateMachine;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoChooser {
	
	// position selection
	public static final int POS_UNDEFINED = 0;
	public static final int LEFT_POSITION = 1;
	public static final int CENTER_POSITION = 2;
	public static final int RIGHT_POSITION = 3;
	
	//  action type selection
	public static final int DO_NOTHING = 0;
	public static final int DRIVE_FORWARD = 1;
	public static final int CUBE_OPS_STD = 2;
	public static final int CUBE_OPS_ADV = 3;
	public static final int LIFT_FOREVER = 4;
		
	// internal selection class used for SendableChooser only
	public class ModeSelection {
		public int mode = DO_NOTHING;
		ModeSelection(int mode) {
			this.mode = mode;
		}
	}
	
	int mode;
	
	private SendableChooser<ModeSelection> chooser_position;
	private SendableChooser<ModeSelection> chooser_action;

	public AutoChooser() {

		// position chooser setup
		chooser_position = new SendableChooser<ModeSelection>();	
		chooser_position.addDefault("POS_UNDEFINED", new ModeSelection(POS_UNDEFINED));
		chooser_position.addObject("LEFT_POSITION", new ModeSelection(LEFT_POSITION));
		chooser_position.addObject("CENTER_POSITION", new ModeSelection(CENTER_POSITION));
		chooser_position.addObject("RIGHT_POSITION", new ModeSelection(RIGHT_POSITION));
		SmartDashboard.putData("AutoChooser_Position", chooser_position);

		// action chooser setup
		chooser_action = new SendableChooser<ModeSelection>();
		chooser_action.addDefault("DO_NOTHING", new ModeSelection(DO_NOTHING));
		chooser_action.addObject("DRIVE_FORWARD", new ModeSelection(DRIVE_FORWARD));
		chooser_action.addObject("CUBE_OPS_STD", new ModeSelection(CUBE_OPS_STD));
		chooser_action.addObject("CUBE_OPS_ADV", new ModeSelection(CUBE_OPS_ADV));
		chooser_action.addObject("LIFT_FOREVER", new ModeSelection(LIFT_FOREVER));
		SmartDashboard.putData("AutoChooser_Action", chooser_action);
	}
	
	public int getAction() {
		
		// check action chooser
		ModeSelection action_selection = chooser_action.getSelected();
		if (action_selection.mode != DO_NOTHING)
			return action_selection.mode;	

		// default - do nothing
		return DO_NOTHING;
	}
	
	public int getPosition() {
		// check position chooser
		ModeSelection pos_selection = chooser_position.getSelected();
		if (pos_selection.mode != DO_NOTHING)
			return pos_selection.mode;	

		// default - position undefined
		return POS_UNDEFINED;		
	}

}
