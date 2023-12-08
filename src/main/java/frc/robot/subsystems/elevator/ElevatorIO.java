package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj2.command.Command;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
  }
  public boolean isZeroed = false;

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ElevatorIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

  /** Run closed loop at the specified velocity. */
  public default void setVelocity(double velocityRadPerSec) {}

  public default void goToPosition(double position) {}

  public default void reset() {}

  /** Stop in open loop. */
  public default void stop() {}

  public default void periodic() {}

  public default boolean zeroed(){return isZeroed;}

  /** Set velocity PID constants. */
  public default void configurePID(double kP, double kI, double kD) {}
}