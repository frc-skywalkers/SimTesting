package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.LimelightConstants.*;
import frc.robot.Preset;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.Presets;
import frc.robot.subsystems.intake.Intake;
//import frc.robot.subsystems.Lightstrip;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.drive.Drive;

/** Add your docs here. */
public class Macros {

  private final Drive swerve;
  private final Elevator elevator;
  private final Arm arm;
  private final Intake intake;
  private final Limelight limelight;
  public final int conePiece = -1;
  public final int cubePiece = 1;


  public Macros(
      Drive swerve, 
      Elevator elevator, 
      Arm arm, 
      Intake intake, 
      Limelight limelight) {
    
    this.swerve = swerve;
    this.elevator = elevator;
    this.arm = arm;
    this.intake = intake;
    this.limelight = limelight;

  }

  public Command home() {
    return Commands.parallel(
      new HomeElevator(elevator),
      arm.goToPosition(Presets.STOW_PRESET.kArmPos)
    );
  }

  public Command moveToPreset(double elevatorPos, double armPos) {
    return Commands.parallel(
      arm.goToPosition(armPos),
      elevator.goToPosition(elevatorPos)
    );
  }

  public Command moveToPreset(Preset preset) {
    return Commands.parallel(
      arm.goToPosition(preset.kArmPos),
      elevator.goToPosition(preset.kElevatorPos)
    );
  }


  public Command stow() {
    return moveToPreset(
      Presets.STOW_PRESET.kElevatorPos, 
      Presets.STOW_PRESET.kArmPos);
  }

  public Command setCubeMode() {
    return Commands.runOnce(() -> {
      intake.setMode(cubePiece);
    });
  }

  public Command setConeMode() {
    return Commands.runOnce(() -> {
      intake.setMode(conePiece);
    });
  }

  public Command groundIntake(boolean intakeOn, int m) {
    return Commands.sequence(
      Commands.runOnce(() -> intake.setMode(m)),
      groundIntake(),
      new IntakePiece(intake, m).unless(() -> !intakeOn)
    );
  }

  public Command substationIntake(boolean intakeOn, int m) {
    return Commands.sequence(
      Commands.runOnce(() -> intake.setMode(m)),
      substationIntake(),
      new IntakePiece(intake).unless(() -> !intakeOn)
    );
  }

  public Command cubeGroundIntake() {
    return moveToPreset(
      Presets.GROUND_INTAKE_CUBE_PRESET
    );
  }

  public Command coneGroundIntake() {
    return moveToPreset(
      Presets.GROUND_INTAKE_CONE_PRESET
    );
  }

  public Command groundIntake() {
    return Commands.runOnce(() -> {
      int m = intake.getMode();
      if (m == conePiece) {
        CommandScheduler.getInstance().schedule(moveToPreset(Presets.GROUND_INTAKE_CONE_PRESET));
      } else if (m == cubePiece) {
        CommandScheduler.getInstance().schedule(moveToPreset(Presets.GROUND_INTAKE_CUBE_PRESET));
      } else {
        System.out.println("GROUND INTAKE ERROR");
      }
    });
  }

  public Command groundIntake(int m) {
    return Commands.runOnce(() -> {
      if (m == conePiece) {
        CommandScheduler.getInstance().schedule(moveToPreset(Presets.GROUND_INTAKE_CONE_PRESET));
      } else if (m == cubePiece) {
        CommandScheduler.getInstance().schedule(moveToPreset(Presets.GROUND_INTAKE_CUBE_PRESET));
      } else {
        System.out.println("GROUND INTAKE ERROR");
      }
    }
    );
  }

  public Command substationIntake() {
      return Commands.runOnce(() -> {
        int m = intake.getMode();
        if (m == conePiece) {
          CommandScheduler.getInstance().schedule(moveToPreset(Presets.SUBSTATION_INTAKE_CONE_PRESET));
        } else if (m == cubePiece) {
          CommandScheduler.getInstance().schedule(moveToPreset(Presets.SUBSTATION_INTAKE_CUBE_PRESET));
        } else {
          System.out.println("SUBSTATION INTAKE ERROR");
        }
      }
    );
  }

  public Command singleSubstationIntake() {
    return Commands.runOnce(() -> {
      int m = intake.getMode();
      if (m == conePiece) {
        CommandScheduler.getInstance().schedule(moveToPreset(Presets.SINGLE_SUBSTATION_CONE));
      } else if (m == cubePiece) {
        CommandScheduler.getInstance().schedule(moveToPreset(Presets.SINGLE_SUBSTATION_CUBE));
      } else {
        System.out.println("SUBSTATION INTAKE ERROR");
      }
    }
  );
}

  public Command cube2ndStage() {
    return moveToPreset(
      Presets.CUBE_2ND_STAGE_PRESET.kElevatorPos, 
      Presets.CUBE_2ND_STAGE_PRESET.kArmPos);
  }

  public Command cube3rdStage() {
    return moveToPreset(
      Presets.CUBE_3RD_STAGE_PRESET.kElevatorPos, 
      Presets.CUBE_3RD_STAGE_PRESET.kArmPos);
  }

  public Command cone2ndStage() {
    return moveToPreset(
      Presets.CONE_2ND_STAGE_PRESET.kElevatorPos, 
      Presets.CONE_2ND_STAGE_PRESET.kArmPos);
  }

  public Command cone3rdStage() {
    return moveToPreset(
      Presets.CONE_3RD_STAGE_PRESET.kElevatorPos, 
      Presets.CONE_3RD_STAGE_PRESET.kArmPos);
  }

  public Command intake() {
    return new IntakePiece(intake).andThen(Commands.run(() -> {
      intake.holdObject();
    }, intake).until(() -> {return intake.getVelocity() > (IntakeConstants.kHoldSpeed / 2);}));
  }

  /*
  public CommandBase intake(Lightstrip lightstrip) {
    return new IntakePiece(intake, lightstrip).andThen(Commands.run(() -> {
      intake.holdObject();
    }, intake).until(() -> {return intake.getSpeed() > (IntakeConstants.kHoldSpeed / 2);}));
  }
  */

  public Command intake(int m) {
    return new IntakePiece(intake, m).andThen(Commands.run(() -> {
      intake.holdObject();
    }, intake).until(() -> {return intake.getVelocity() > (IntakeConstants.kHoldSpeed / 2);}));
  }

  public Command outtake() {
    return new OuttakePiece(intake).withTimeout(1);
  }

  public Command outtake(int m) {
    return new OuttakePiece(intake, m).withTimeout(1);
  }

  /*
  public CommandBase alignCone2ndStage() {
    return new AlignCone(swerve, limelight, SecondStageConeConstants.targetXMeters, SecondStageConeConstants.targetYMeters, SecondStageConeConstants.targetRDeg);
  }
  */

  /*
  public CommandBase scoreCone2ndStage() {
    return Commands.sequence(
      new TurnAngle(swerve, 0),
      alignCone2ndStage(),
      cone2ndStage(),
      moveToPreset(Presets.CONE_2ND_STAGE_PRESET.kElevatorPos, 0),
      outtake(),
      stow()
    );
  }
  */

  /*
  public CommandBase scoreCone3rdStage() {
    return Commands.sequence(
      new TurnAngle(swerve, 0),
      alignCone2ndStage(),
      cone3rdStage(),
      outtake(),
      stow()
    );
  }
  */

  public Command general2ndStage() {
    return Commands.runOnce(() -> {
        int m = intake.getMode();
        System.out.println("INTAKE MODE 2: " + m);
        if (m == conePiece) {
          CommandScheduler.getInstance().schedule(cone2ndStage());
        } else if (m == cubePiece) {
          CommandScheduler.getInstance().schedule(cube2ndStage());
        } else {
          System.out.println("GENERAL 2ND STAGE ERROR");
          System.out.println("NO PIECE INSIDE INTAKE");
        }
      }
    );
  }

  public Command general3rdStage() {
    return Commands.runOnce(() -> {
        int m = intake.getMode();
        System.out.println("INTAKE MODE 3: " + m);
        if (m == conePiece) {
          CommandScheduler.getInstance().schedule(cone3rdStage());
        } else if (m == cubePiece) {
          CommandScheduler.getInstance().schedule(cube3rdStage());
        } else {
          System.out.println("GENERAL 3RD STAGE ERROR");
          System.out.println("NO PIECE INSIDE INTAKE");
        }
      }
    );
  }

}