package NetworkComm;

//import edu.wpi.first.wpilibj.networktables.NetworkTable;  // deprecated in 2018
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class InputOutputComm {
	
	private static boolean initialized = false;
	
	public static void initialize() {
		if (initialized)
			return;
		
        //table = NetworkTable.getTable("InputOutput1778/DataTable");    // deprecated in 2018
		
		tableInstance = NetworkTableInstance.getDefault();
		table = tableInstance.getTable("InputOutput1778/DataTable");		
   
        initialized = true;
	}
			
    public static enum LogTable { kMainLog, kRPICommLog, kDriveLog };
			  
    // instance data and methods
    private static NetworkTableInstance tableInstance;
    private static NetworkTable table;
    
    public static void putBoolean(LogTable log, String key, boolean value) {
    	    	
    	if (table != null)
    		table.getEntry(key).setBoolean(value);
    	else
    		System.out.println("No network table to write to!!");
    }
    
    public static void putDouble(LogTable log, String key, double value) {
    	if (table != null)
    		table.getEntry(key).setDouble(value);
    	else
    		System.out.println("No network table to write to!!");
    }
    
    public static void putInt(LogTable log, String key, int value) {
    	if (table != null)
    		table.getEntry(key).setNumber(value);
    	else
    		System.out.println("No network table to write to!!");
    }
    
    public static void putString(LogTable log, String key, String outputStr) {
    	if (table != null)
    		table.getEntry(key).setString(outputStr);
    	else
    		System.out.println("No network table to write to!!");
    }
    
    public static void deleteKey(String key)
    {
    	if (table != null)
    		table.delete(key);
    	else
    		System.out.println("No network table to write to!!");
    	
    }
}
