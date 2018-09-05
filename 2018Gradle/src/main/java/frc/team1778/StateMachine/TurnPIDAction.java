package frc.team1778.StateMachine;

import frc.team1778.Systems.DriveAssembly;
import frc.team1778.Systems.NavXSensor;
import java.util.prefs.Preferences;

public class TurnPIDAction extends Action {

  private double angleToTurn = 0.0;
  private double speedToTurn = 0.3;
  private boolean resetGyro = true;

  public TurnPIDAction(double angleToTurn, double speed, boolean resetGyro) {
    this.name = "<Turn Action>";
    this.angleToTurn = angleToTurn; // absolute heading to use if not resetting gyro
    this.speedToTurn = speed;
    this.resetGyro = resetGyro;

    DriveAssembly.initialize();
    NavXSensor.initialize();
  }

  public TurnPIDAction(String name, double angleToTurn, double speed, boolean resetGyro) {
    this.name = name;
    this.angleToTurn = angleToTurn; // absolute heading to use if not resetting gyro
    this.speedToTurn = speed;
    this.resetGyro = resetGyro;

    DriveAssembly.initialize();
    NavXSensor.initialize();
  }

  // action entry
  public void initialize() {

    // if we're not resetting the gyro, we'll want to see what angle it is to start
    if (resetGyro) NavXSensor.reset();

    // initialize motor assembly for auto
    DriveAssembly.autoPidTurnStart(angleToTurn, speedToTurn);

    super.initialize();
  }

  // called periodically
  public void process() {

    DriveAssembly.autoPidTurnProcess();

    super.process();
  }

  // action cleanup and exit
  public void cleanup() {
    // do some drivey cleanup

    DriveAssembly.autoPidTurnStop();

    // cleanup base class
    super.cleanup();
  }

  public void persistWrite(int counter, Preferences prefs) {

    // create node for action
    Preferences actionPrefs = prefs.node(counter + "_" + this.name);

    // store action details
    actionPrefs.put("class", this.getClass().toString());
    actionPrefs.putDouble("angleToTurn", this.angleToTurn);
    actionPrefs.putDouble("speedToTurn", this.speedToTurn);
  }
}
