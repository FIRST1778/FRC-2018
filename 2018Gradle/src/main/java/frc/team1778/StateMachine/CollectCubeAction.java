package frc.team1778.StateMachine;

import frc.team1778.NetworkComm.InputOutputComm;
import frc.team1778.Systems.CubeManagement;

public class CollectCubeAction extends Action {

  private String name;
  private double strength = CubeManagement.COLLECTOR_IN_AUTO_STRENGTH;

  public CollectCubeAction() {
    this.name = "<Collect Cube Action>";

    CubeManagement.initialize();
    InputOutputComm.initialize();
  }

  public CollectCubeAction(String name, double strength) {
    this.name = name;
    this.strength = strength;

    CubeManagement.initialize();
    InputOutputComm.initialize();
  }

  // action entry
  public void initialize() {

    // do some clamp initialization, start the collector motors to collect the cube
    CubeManagement.collectCube(strength);

    super.initialize();
  }

  // called periodically
  public void process() {

    // do some stuff - nothing specific required for flipper
    super.process();
  }

  // state cleanup and exit
  public void cleanup() {
    // do some drivey cleanup

    CubeManagement.autoStop();

    // cleanup base class
    super.cleanup();
  }
}
