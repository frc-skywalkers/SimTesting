package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  private final SimpleMotorFeedforward ffModelUp;
  private final SimpleMotorFeedforward ffModelDown;
  private final ProfiledPIDController pid;
  public boolean enabled = false;
  public double goal;

  /** Creates a new Elevator. */
  public Arm(ArmIO io) {
    this.io = io;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.currentMode) {
      case REAL: //idk should these all be with the same constants?
        ffModelUp = new SimpleMotorFeedforward(ArmConstants.kSUp, ArmConstants.kVUp);
        ffModelDown = new SimpleMotorFeedforward(ArmConstants.kSDown, ArmConstants.kVDown);
        pid = new ProfiledPIDController(ArmConstants.kP, 0, 0, new TrapezoidProfile.Constraints(ArmConstants.kMaxVel, ArmConstants.kMaxAcc));
        break;
      case REPLAY: //?
        ffModelUp = new SimpleMotorFeedforward(ArmConstants.kSUp, ArmConstants.kVUp);
        ffModelDown = new SimpleMotorFeedforward(ArmConstants.kSDown, ArmConstants.kVDown);
        pid = new ProfiledPIDController(ArmConstants.kPsim, 0, 0, new TrapezoidProfile.Constraints(ArmConstants.kMaxVel, ArmConstants.kMaxAcc));
        break;
      case SIM:
        ffModelUp = new SimpleMotorFeedforward(ArmConstants.kSUp, ArmConstants.kVUp);
        ffModelDown = new SimpleMotorFeedforward(ArmConstants.kSDown, ArmConstants.kVDown);
        pid = new ProfiledPIDController(ArmConstants.kP, 0, 0, new TrapezoidProfile.Constraints(ArmConstants.kMaxVel, ArmConstants.kMaxAcc));
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
    Logger.processInputs("Arm", inputs);
    // Log elevator speed in RPM
    Logger.recordOutput("ArmSpeedRPM", getVelocityRPM());

    if (enabled) {
      //System.out.println("ajskdl"); //working
      pid.setGoal(new State(goal, 0));
      pid.setTolerance(0.07, 0.07);
      double volts;
      double error = Math.abs(goal-inputs.positionRad);

      if (pid.getSetpoint().velocity>0){ //?
        volts = pid.calculate(inputs.positionRad) + ffModelUp.calculate(pid.getSetpoint().velocity);
      } else {
        volts = pid.calculate(inputs.positionRad) + ffModelDown.calculate(pid.getSetpoint().velocity);
      }

      if (error<0.07){ //put this in constants later, also fix. shouldnt need this probably
        volts = 0;
      }

      io.setVoltage(volts); //this is where appliedvolts comes from, clamped 8
      Logger.recordOutput("calculated volts", volts);
      Logger.recordOutput("Arm goal", goal);
      Logger.recordOutput("error", error); }
    else {
      io.setVoltage(0);
    }
    }

  public Command goToPosition(double goal) {
    BooleanSupplier sup = () -> pid.atGoal();
    return Commands.runOnce(()-> {
      enabled = true;
      this.goal = goal;
    }, this).andThen(Commands.waitUntil(sup)).andThen(() -> {enabled = false;});    
    }

    /*
    pid.setGoal(new State(goal, 0));
    //pid.setTolerance(10, 0.01);
    double volts;
    double error = Math.abs(goal-inputs.positionRad);
    if (pid.getSetpoint().velocity > 0){
      volts = pid.calculate(inputs.positionRad) + ffModelUp.calculate(pid.getSetpoint().velocity);
    } else {
      volts = pid.calculate(inputs.positionRad) + ffModelDown.calculate(pid.getSetpoint().velocity);
    }
    if (error<5){
      volts = 0;
    }
    io.setVoltage(volts);
    Logger.recordOutput("Elevator goal", goal);
    Logger.recordOutput("error", error);
    */
  

  /** Stops the flywheel. */
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