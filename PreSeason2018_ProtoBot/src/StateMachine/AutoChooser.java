package StateMachine;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoChooser {
	
	// drive networks
	public static final int DO_NOTHING = 0;
	public static final int DRIVE_FORWARD = 1;
	
	// gear networks
	public static final int DEPOSIT_GEAR_LEFT = 2;
	public static final int DEPOSIT_GEAR_CENTER = 3;
	public static final int DEPOSIT_GEAR_RIGHT = 4;
	
	// debug networks
	public static final int TURNING_FOREVER = 5;
	public static final int PACING_FOREVER = 6;
	public static final int FOLLOW_FOREVER = 7;

	// legacy networks
	public static final int DRIVE_AND_SHOOT_BLUE_LEFT = 8;
	public static final int DRIVE_AND_SHOOT_RED_RIGHT = 9;
	public static final int DEPOSIT_GEAR_AND_SHOOT_RED_CENTER = 10;
	public static final int DEPOSIT_GEAR_AND_SHOOT_BLUE_CENTER = 11;
	public static final int SHOOT_AND_DRIVE_BLUE_LEFT = 12;
	public static final int SHOOT_AND_DRIVE_RED_RIGHT = 13;
	public static final int DRIVE_AND_SHOOT_NEAR = 14;
	public static final int DRIVE_AND_SHOOT_MEDIUM = 15;
	
	// internal selection class used for SendableChooser only
	public class ModeSelection {
		public int mode = DO_NOTHING;
		ModeSelection(int mode) {
			this.mode = mode;
		}
	}
	
	int mode;
	private SendableChooser<ModeSelection> chooser_basic;

	public AutoChooser() {

		chooser_basic = new SendableChooser<ModeSelection>();
		
		chooser_basic.addDefault("DO_NOTHING", new ModeSelection(DO_NOTHING));
		chooser_basic.addObject("DRIVE_FORWARD", new ModeSelection(DRIVE_FORWARD));
		chooser_basic.addObject("DEPOSIT_GEAR_LEFT", new ModeSelection(DEPOSIT_GEAR_LEFT));
		chooser_basic.addObject("DEPOSIT_GEAR_CENTER", new ModeSelection(DEPOSIT_GEAR_CENTER));
		chooser_basic.addObject("DEPOSIT_GEAR_RIGHT", new ModeSelection(DEPOSIT_GEAR_RIGHT));
		chooser_basic.addObject("DRIVE_AND_SHOOT_BLUE_LEFT", new ModeSelection(DRIVE_AND_SHOOT_BLUE_LEFT));
		chooser_basic.addObject("DRIVE_AND_SHOOT_RED_RIGHT", new ModeSelection(DRIVE_AND_SHOOT_RED_RIGHT));
		chooser_basic.addObject("DEPOSIT_GEAR_AND_SHOOT_RED_CENTER", new ModeSelection(DEPOSIT_GEAR_AND_SHOOT_RED_CENTER));
		chooser_basic.addObject("DEPOSIT_GEAR_AND_SHOOT_BLUE_CENTER", new ModeSelection(DEPOSIT_GEAR_AND_SHOOT_BLUE_CENTER));
		chooser_basic.addObject("TURNING_FOREVER", new ModeSelection(TURNING_FOREVER));
		chooser_basic.addObject("PACING_FOREVER", new ModeSelection(PACING_FOREVER));
		chooser_basic.addObject("FOLLOW_FOREVER", new ModeSelection(FOLLOW_FOREVER));
				
		SmartDashboard.putData("AutoChooser_Basic", chooser_basic);
	}
	
	public int getAutoChoice() {
		
		// scans choosers in order, returns the first to have action
		
		// check basic
		ModeSelection selection = chooser_basic.getSelected();
		if (selection.mode != DO_NOTHING)
			return selection.mode;	

		// default - do nothing
		return DO_NOTHING;
	}

}
