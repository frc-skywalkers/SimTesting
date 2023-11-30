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
    switch (Constants.currentMode) { //currentMode needs to be changed in Constants depending on situation
      case REAL: 
        ffModelUp = new SimpleMotorFeedforward(ArmConstants.kSUp, ArmConstants.kVUp);
        ffModelDown = new SimpleMotorFeedforward(ArmConstants.kSDown, ArmConstants.kVDown);
        pid = new ProfiledPIDController(ArmConstants.kP, 0, 0, new TrapezoidProfile.Constraints(ArmConstants.kMaxVel, ArmConstants.kMaxAcc));
        break;
      case REPLAY: 
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
    Logger.recordOutput("ArmSpeedRPM", getVelocityRPM()); //goes under RealOutputs
    Logger.recordOutput("ArmPosRad", inputs.positionRad);
    Logger.recordOutput("ArmVolts", inputs.appliedVolts);
    Logger.recordOutput("ArmCurrent", inputs.currentAmps);


    if (enabled) { //only true while goToPosition is running
      pid.setGoal(new State(goal, 0)); 
      pid.setTolerance(0.07, 0.07);
      double volts;
      double error = Math.abs(goal-inputs.positionRad);

      if (pid.getSetpoint().velocity>0){ //signs of velocities on the way to the goal to see if elevator is going up or down
        volts = pid.calculate(inputs.positionRad) + ffModelUp.calculate(pid.getSetpoint().velocity);
      } else {
        volts = pid.calculate(inputs.positionRad) + ffModelDown.calculate(pid.getSetpoint().velocity);
      }

      
      if (error<0.07){ //v redundant but there's a small spike for some reason when this is removed
        volts = 0;
      }
      

      io.setVoltage(volts);
    }
    else {
      io.setVoltage(0);
    }
    }

  public Command goToPosition(double goal) {
    BooleanSupplier sup = () -> pid.atGoal();
    return Commands.runOnce(()-> {
      enabled = true; //turns on all the calculations
      this.goal = goal;
    }, this).andThen(Commands.waitUntil(sup)).andThen(() -> {enabled = false;}); //turns off once goal is reached 
    }

  //stops arm
  public void stop() {
    io.stop();
  }

  public void runVelocity(double velocity) {
    io.setVelocity(velocity);
  }

  //converts to RPM for logging
  public double getVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
  }

  public void runCharacterizationVolts(double volts) {
    io.setVoltage(volts);
  }

  //avg velocity in radians/sec
  public double getCharacterizationVelocity() {
    return inputs.velocityRadPerSec;
  }
}