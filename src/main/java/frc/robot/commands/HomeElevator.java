package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Dashboard;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.elevator.Elevator;

public class HomeElevator extends Command {
  /** Creates a new HomeElevator. */

  private final Elevator elevator;

  public HomeElevator(Elevator elevator) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevator = elevator;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //elevator.disableSoftLimits();
    Dashboard.ElevatorDash.Driver.putBoolean("Zeroed", false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.runVelocity(ElevatorConstants.kHomingSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Dashboard.ElevatorDash.Debugging.putString("Ended Home", "yes");
    elevator.stop();
    elevator.reset(); //only resets the real elevator, isZeroed only applies to real elevator (sim always zeroed)
    elevator.isZeroed = true;
    Dashboard.ElevatorDash.Driver.putBoolean("Zeroed", true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return elevator.getCurrent() > ElevatorConstants.kCurrentThreshold;
  }
}