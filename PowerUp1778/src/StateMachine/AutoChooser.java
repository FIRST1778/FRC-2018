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
	public static final int CUBE_OPS = 2;
	public static final int LIFT_FOREVER = 3;
	public static final int TURN_FOREVER = 4;
	public static final int PACE_FOREVER = 5;
	public static final int TURN_ONCE = 6;
	
	//  action priority type selection (left or right position only)
	public static final int SCALE = 0;
	public static final int SWITCH = 1;

	//  priority type selection (remote scale actions only)
	public static final int REMOTE_SCALE_CUBE_DROP = 0;
	public static final int REMOTE_SCALE_STANDBY = 1;

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
	private SendableChooser<ModeSelection> chooser_right_left_priority;
	private SendableChooser<ModeSelection> chooser_remote_scale_action;	

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
		chooser_action.addObject("CUBE_OPS", new ModeSelection(CUBE_OPS));
		chooser_action.addObject("LIFT_FOREVER", new ModeSelection(LIFT_FOREVER));
		chooser_action.addObject("TURN_FOREVER", new ModeSelection(TURN_FOREVER));
		chooser_action.addObject("PACE_FOREVER", new ModeSelection(PACE_FOREVER));
		chooser_action.addObject("TURN_ONCE", new ModeSelection(TURN_ONCE));
		SmartDashboard.putData("AutoChooser_Action", chooser_action);
		
		// strategy chooser setup - switch or scale (for left or right only)
		chooser_right_left_priority = new SendableChooser<ModeSelection>();
		chooser_right_left_priority.addDefault("SCALE", new ModeSelection(SCALE));
		chooser_right_left_priority.addObject("SWITCH", new ModeSelection(SWITCH));
		SmartDashboard.putData("AutoChooser_Right_Left_Priority", chooser_right_left_priority);
				
		// strategy chooser setup - remote cube drop or remote standby (remote scale only)
		chooser_remote_scale_action = new SendableChooser<ModeSelection>();
		chooser_remote_scale_action.addDefault("REMOTE_SCALE_CUBE_DROP", new ModeSelection(REMOTE_SCALE_CUBE_DROP));
		chooser_remote_scale_action.addObject("REMOTE_SCALE_STANDBY", new ModeSelection(REMOTE_SCALE_STANDBY));
		SmartDashboard.putData("AutoChooser_Remote_Scale_Action", chooser_remote_scale_action);
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
		if (pos_selection.mode != POS_UNDEFINED)
			return pos_selection.mode;	

		// default - position undefined
		return POS_UNDEFINED;		
	}
	
	public int getRightLeftPriority() {
		// check right-left priority chooser
		ModeSelection right_left_priority_selection = chooser_right_left_priority.getSelected();
		if (right_left_priority_selection.mode != SCALE)
			return right_left_priority_selection.mode;	

		// default - scale is first priority
		return SCALE;		
	}

	public int getRemoteScaleAction() {
		// check remote scale action chooser
		ModeSelection remote_scale_action = chooser_remote_scale_action.getSelected();
		if (remote_scale_action.mode != REMOTE_SCALE_CUBE_DROP)
			return remote_scale_action.mode;	

		// default - remote scale cube drop is default
		return REMOTE_SCALE_CUBE_DROP;		
	}

}
