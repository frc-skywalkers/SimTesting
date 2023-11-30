package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private final SimpleMotorFeedforward ffModelUp;
  private final SimpleMotorFeedforward ffModelDown;
  private final ProfiledPIDController pid;
  public boolean enabled = false;
  public double goal;

  /** Creates a new Elevator. */
  public Elevator(ElevatorIO io) {
    this.io = io;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.currentMode) {
      case REAL: //can tune constants differently
        ffModelUp = new SimpleMotorFeedforward(ElevatorConstants.kSUp, ElevatorConstants.kVUp);
        ffModelDown = new SimpleMotorFeedforward(ElevatorConstants.kSDown, ElevatorConstants.kVDown);
        pid = new ProfiledPIDController(ElevatorConstants.kP, 0, 0, new TrapezoidProfile.Constraints(ElevatorConstants.kMaxVel, ElevatorConstants.kMaxAcc));
        break;
      case REPLAY:
        ffModelUp = new SimpleMotorFeedforward(ElevatorConstants.kSUp, ElevatorConstants.kVUp);
        ffModelDown = new SimpleMotorFeedforward(ElevatorConstants.kSDown, ElevatorConstants.kVDown);
        pid = new ProfiledPIDController(ElevatorConstants.kP, 0, 0, new TrapezoidProfile.Constraints(ElevatorConstants.kMaxVel, ElevatorConstants.kMaxAcc));
        break;
      case SIM:
        ffModelUp = new SimpleMotorFeedforward(ElevatorConstants.kSUp, ElevatorConstants.kVUp);
        ffModelDown = new SimpleMotorFeedforward(ElevatorConstants.kSDown, ElevatorConstants.kVDown);
        pid = new ProfiledPIDController(ElevatorConstants.kP, 0, 0, new TrapezoidProfile.Constraints(ElevatorConstants.kMaxVel, ElevatorConstants.kMaxAcc));
        break;
      default:
        ffModelUp = new SimpleMotorFeedforward(0.0, 0.0);
        ffModelDown = new SimpleMotorFeedforward(0.0, 0.0);
        pid = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));
        break;
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
    // Log elevator speed in RPM
    Logger.recordOutput("ElevatorSpeedRPM", getVelocityRPM());
    Logger.recordOutput("ElevatorPosRad", inputs.positionRad);
    Logger.recordOutput("ElevatorVolts", inputs.appliedVolts);
    Logger.recordOutput("ElevatorCurrent", inputs.currentAmps);

    if (enabled) {
      //System.out.println("ajskdl"); //working
      pid.setGoal(new State(goal, 0));
      pid.setTolerance(0.07, 0.07);
      double volts;
      double error = Math.abs(goal-inputs.positionRad);

      if (pid.getSetpoint().velocity>0){ //up or down, voltage adjust for gravity
        volts = pid.calculate(inputs.positionRad) + ffModelUp.calculate(pid.getSetpoint().velocity);
      } else {
        volts = pid.calculate(inputs.positionRad) + ffModelDown.calculate(pid.getSetpoint().velocity);
      }

      if (error<0.07){ //shouldnt need this
        volts = 0;
      }

      io.setVoltage(volts); //this is where appliedvolts comes from, clamped 8
      Logger.recordOutput("calculated volts", volts);
      Logger.recordOutput("Elevator goal", goal);
      Logger.recordOutput("error", error); }
    else {
      io.setVoltage(0);
    }
    }

  public Command goToPosition(double goal) {
    BooleanSupplier sup = () -> pid.atGoal();
    return Commands.runOnce(()-> {
      enabled = true; //when gotoposition is running it calculates voltage, otherwise nothing
      this.goal = goal; //^ might need to make some minor changes before adding direct joystick control
    }, this).andThen(Commands.waitUntil(sup)).andThen(() -> {enabled = false;});    
    }
  
  public void stop() {
    io.stop();
  }

  public void runVelocity(double velocity) {
    io.setVelocity(velocity);
  }

  /** Returns the current velocity in RPM. */
  public double getVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
  }

  /** Runs forwards at the commanded voltage. */
  public void runCharacterizationVolts(double volts) {
    io.setVoltage(volts);
  }

  /** Returns the average drive velocity in radians/sec. */
  public double getCharacterizationVelocity() {
    return inputs.velocityRadPerSec;
  }
}