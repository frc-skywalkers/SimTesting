package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Dashboard;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.lightstripConstants;
import frc.robot.Dashboard.IntakeDash;
import frc.robot.lightstrip.TempLedState;
import frc.robot.subsystems.intake.Intake;
//import frc.robot.subsystems.Lightstrip;

public class IntakePiece extends Command {
  private final Intake intake;

  private boolean finished = false;

  private int stage;

  //private Lightstrip lightstrip = null;

  //private CommandXboxController controller;
  /** Creates a new IntakeMotor. */
  public IntakePiece(Intake rIntake) {
    intake = rIntake;
    
    addRequirements(rIntake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  /* 
  public IntakePiece(Intake rIntake) { //Lightstrip rLightstrip
    intake = rIntake;
    //lightstrip = rLightstrip;
    addRequirements(rIntake);
    // Use addRequirements() here to declare subsystem dependencies.
  }
  */
  public IntakePiece(Intake rIntake, int mode) {
    intake = rIntake;
    intake.setMode(mode);
    addRequirements(rIntake);
  }
  /*
  public IntakePiece(Intake rIntake, Mode mode) {
    intake = rIntake;
    intake.setMode(mode);
    addRequirements(rIntake);
    // Use addRequirements() here to declare subsystem dependencies.
  }
  */

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stage = 0;
    finished = false;
    intake.moveIn();
    intake.stop = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(stage == 0) {
      if(intake.intakeEmpty()) { // waits for power to go up
        stage = 1;
      } else {
        intake.moveIn();
      }
    } else { //starts checking for game pieces once its speed up
      if(intake.pieceHeld()) { // waits for spike when object is intaked
        intake.stop();
        finished = true;
        stage = -1;
      } else {
        intake.moveIn();
      }
    }
    Dashboard.IntakeDash.Debugging.putNumber("Intake Step", stage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(!interrupted) {
      intake.holdObject();
      /*
      if(lightstrip != null) {
        lightstrip.tempColor(lightstripConstants.successSignal);
      }
      */
    } else {
      intake.stop();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished || intake.stop;
  }
}