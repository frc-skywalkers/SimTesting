package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.elevator.Elevator;
import java.util.function.DoubleSupplier;

public class ElevatorCommands {
  private static final double DEADBAND = 0.1;

  private ElevatorCommands() {}

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickElevator(
      Elevator elevator,
      DoubleSupplier xSupplier) {
    return Commands.run(
        () -> {
          // Apply deadband
          double linearMagnitude =
              MathUtil.applyDeadband(
                  xSupplier.getAsDouble(), DEADBAND);
        
          // Square values
          linearMagnitude = linearMagnitude * linearMagnitude;

          // Convert to field relative speeds & send command
          elevator.runVelocity(linearMagnitude);
        },
        elevator);
    }

  public Command goToPosition(Elevator elevator, double position){
    return Commands.runOnce(() ->
        elevator.goToPosition(position));
  }
}
  
  /*
  public Command goToPosition(double position) {
    return Commands.runOnce(() -> {
      System.out.println(position);
      this.setGoal(position);
      this.enable();
    }, this).andThen(Commands.waitUntil(this::atGoal));
  }
  */