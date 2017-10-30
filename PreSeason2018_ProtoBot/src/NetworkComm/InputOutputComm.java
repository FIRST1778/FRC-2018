package NetworkComm;

//import edu.wpi.first.wpilibj.networktables.NetworkTable;  // deprecated in 2018
import edu.wpi.first.networktables.NetworkTable;

public class InputOutputComm {
	
    // singleton class elements (ensures only one instance of this class)
	private static final InputOutputComm instance = new InputOutputComm();
    
	private InputOutputComm() {
        //table = NetworkTable.getTable("InputOutput1778/DataTable");
        table = new NetworkTable(null, "InputOutput1778/DataTable");
	}
		
	public static InputOutputComm GetInstance() {
		return instance;
	}
	
    public static enum LogTable { kMainLog, kRPICommLog, kDriveLog };
			  
    // instance data and methods
    private NetworkTable table;
    
    public void putBoolean(LogTable log, String key, boolean value) {
    	    	
    	if (table != null)
    		table.getEntry(key).setBoolean(value);
    	else
    		System.out.println("No network table to write to!!");
    }
    
    public void putDouble(LogTable log, String key, double value) {
    	if (table != null)
    		table.getEntry(key).setDouble(value);
    	else
    		System.out.println("No network table to write to!!");
    }
    
    public void putInt(LogTable log, String key, int value) {
    	if (table != null)
    		table.getEntry(key).setNumber(value);
    	else
    		System.out.println("No network table to write to!!");
    }
    
    public void putString(LogTable log, String key, String outputStr) {
    	if (table != null)
    		table.getEntry(key).setString(outputStr);
    	else
    		System.out.println("No network table to write to!!");
    }
    
    public void deleteKey(String key)
    {
    	if (table != null)
    		table.delete(key);
    	else
    		System.out.println("No network table to write to!!");
    	
    }
}
