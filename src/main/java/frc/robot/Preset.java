package frc.robot;

/** Add your docs here. */
public class Preset {

  public final double kElevatorPos;
  public final double kArmPos;

  public Preset(double armPos, double elevatorPos) {
    this.kElevatorPos = elevatorPos;
    this.kArmPos = armPos;
  }
}