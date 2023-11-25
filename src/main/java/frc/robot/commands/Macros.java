package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import java.util.function.DoubleSupplier;

public class Macros {
  private final Drive drive;
  private final Arm arm;
  private final Elevator elevator;
  private final Intake intake;
  
  public Macros(Drive drive, Arm arm, Elevator elevator, Intake intake) {
    this.drive = drive;
    this.arm = arm;
    this.elevator = elevator;
    this.intake = intake;
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */

  public Command moveToPreset(double elevatorPos, double armPos){
    return Commands.parallel(
      arm.goToPosition(armPos),
      elevator.goToPosition(elevatorPos)
    );
  }
}
