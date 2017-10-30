package Systems;

import NetworkComm.InputOutputComm;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;

public class RioDuinoAssembly {
	
    // singleton class elements (ensures only one instance of this class)
	private static final RioDuinoAssembly instance = new RioDuinoAssembly();
    
	private RioDuinoAssembly() {
		i2cBus = new I2C(I2C.Port.kMXP, 4);	
		
		setTeamColor();
	}
		
	public static RioDuinoAssembly GetInstance() {
		return instance;
	}

	public static enum Color { Black, Red, Blue, Yellow, Orange, Green, Purple, Grey};
	
	// particulars about the team number and color
	private DriverStation.Alliance dsTeamColor;
	private int dsTeamLocation;

	private Color teamColor;

	private I2C i2cBus;
		
	public void setTeamColor(Color col) {
		teamColor = col;
		sendColor(teamColor);
	}
	
	public void setTeamColor() {
		dsTeamColor = DriverStation.getInstance().getAlliance();
		dsTeamLocation = DriverStation.getInstance().getLocation();

		 if (dsTeamColor == DriverStation.Alliance.Red)
			 teamColor = RioDuinoAssembly.Color.Red; 
		 else
			 teamColor = RioDuinoAssembly.Color.Blue;

		 sendTeamColor(teamColor);
	}
		
	public void autonomousInit() {
	
		setTeamColor();
		//SendString("colorGreen");
		SendString("autoInit");
	}
	
	public void teleopInit() {
		
		setTeamColor();
		//SendString("colorGreen");
		SendString("teleopInit");
	}
	
	public void testInit() {

		setTeamColor();		
		//SendString("colorOrange");		
		SendString("testInit");
	}
	
	public void disabledInit() {		
		SendString("disabledInit");
	}
		
	public void sendColor(Color col)
	{
		switch (col) {
		case Red:
			SendString("colorRed");
			break;
		case Blue:
			SendString("colorBlue");
			break;
		case Yellow:
			SendString("colorYellow");
			break;
		case Green:
			SendString("colorGreen");
			break;
		case Purple:
			SendString("colorPurple");
			break;
		case Grey:
			SendString("colorGrey");
			break;
		case Black:
		default:
			SendString("colorBlack");
			break;
		}
	}
	
	private void sendTeamColor(Color col)
	{
		switch (col) {
		case Red:
			SendString("teamRed");
			break;
		case Blue:
		default:
			SendString("teamBlue");
			break;
		}
	}
	
	public void SendStateChange(char state)
	{		
		i2cBus.write(0x02, state);
	}
	
	public void SendString(String writeStr)
	{
				
		char[] CharArray = writeStr.toCharArray();
		byte[] WriteData = new byte[CharArray.length];
		for (int i = 0; i < CharArray.length; i++) {
			WriteData[i] = (byte) CharArray[i];
		}
		i2cBus.transaction(WriteData, WriteData.length, null, 0);

	}
}
