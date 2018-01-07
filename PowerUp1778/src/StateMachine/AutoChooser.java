package StateMachine;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoChooser {
	
	// position selection
	public static final int POS_UNDEFINED = 0;
	public static final int LEFT_POSITION = 1;
	public static final int CENTER_POSITION = 2;
	public static final int RIGHT_POSITION = 3;
	
	// drive & cube placement actions
	public static final int DO_NOTHING = 0;
	public static final int DRIVE_FORWARD = 1;
	public static final int DEPOSIT_CUBE_SWITCH = 2;
	public static final int DEPOSIT_CUBE_SCALE = 3;
	public static final int DEPOSIT_CUBE_OPP_SCALE = 4;
		
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
		chooser_position.addDefault("DO_NOTHING", new ModeSelection(POS_UNDEFINED));
		chooser_position.addObject("DRIVE_FORWARD", new ModeSelection(LEFT_POSITION));
		chooser_position.addObject("DEPOSIT_CUBE_SWITCH", new ModeSelection(CENTER_POSITION));
		chooser_position.addObject("DEPOSIT_CUBE_SCALE", new ModeSelection(RIGHT_POSITION));
		SmartDashboard.putData("AutoChooser_Position", chooser_position);

		// action chooser setup
		chooser_action = new SendableChooser<ModeSelection>();
		chooser_action.addDefault("DO_NOTHING", new ModeSelection(DO_NOTHING));
		chooser_action.addObject("DRIVE_FORWARD", new ModeSelection(DRIVE_FORWARD));
		chooser_action.addObject("DEPOSIT_CUBE_SWITCH", new ModeSelection(DEPOSIT_CUBE_SWITCH));
		chooser_action.addObject("DEPOSIT_CUBE_SCALE", new ModeSelection(DEPOSIT_CUBE_SCALE));
		chooser_action.addObject("DEPOSIT_CUBE_OPP_SCALE", new ModeSelection(DEPOSIT_CUBE_OPP_SCALE));				
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
