package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj2.command.Command;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ArmIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

  /** Run closed loop at the specified velocity. */
  public default void setVelocity(double velocityRadPerSec) {}

  public default void goToPosition(double position) {}

  /** Stop in open loop. */
  public default void stop() {}

  public default void periodic() {}

  public default void reset() {}

  /** Set velocity PID constants. */
  public default void configurePID(double kP, double kI, double kD) {}
}