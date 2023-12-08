package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Dashboard;

import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final SimpleMotorFeedforward ffModel;
  private double intakeSpeed = 0;

  public int conePiece = -1;
  public int cubePiece = 1;
  public boolean stop = false;
  

  public Intake(IntakeIO io) {
    this.io = io;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.currentMode) {
      case REAL:
      case REPLAY: //idk
        ffModel = new SimpleMotorFeedforward(0.1, 0.05);
        io.configurePID(1.0, 0.0, 0.0);
        break;
      case SIM:
        ffModel = new SimpleMotorFeedforward(0.0, 0.03);
        io.configurePID(0.5, 0.0, 0.0);
        break;
      default:
        ffModel = new SimpleMotorFeedforward(0.0, 0.0);
        break;
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    // Log speed in RPM
    Logger.recordOutput("IntakeSpeedRPM", getVelocityRPM());
    Logger.recordOutput("IntakePosRad", inputs.positionRad);
    Logger.recordOutput("IntakeVolts", inputs.appliedVolts);
    Logger.recordOutput("IntakeCurrent", inputs.currentAmps);
    Logger.recordOutput("Mode", inputs.modeinput);

    Dashboard.IntakeDash.Debugging.putNumber("Intake Velocity", getVelocityRPM());
    Dashboard.IntakeDash.Debugging.putNumber("Intake Speed", intakeSpeed);
    Dashboard.IntakeDash.Debugging.putNumber("Intake Current", inputs.currentAmps[0]);
    Dashboard.IntakeDash.Driver.putNumber("Intake Mode", inputs.modeinput);
    Dashboard.IntakeDash.Debugging.putNumber(getName(), intakeSpeed); //?
  }

  public void runVelocity(double velocityRPM) {
    intakeSpeed = velocityRPM;
    var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    io.setVelocity(velocityRadPerSec, ffModel.calculate(velocityRadPerSec)); //ok really

    // Log setpoint
    Logger.recordOutput("IntakeSetpointRPM", velocityRPM);
  }

  public void moveIn() {
    io.moveIn(ffModel.calculate(IntakeConstants.kMaxIntakeSpeed)); //multiplier already in io
    intakeSpeed = IntakeConstants.kMaxIntakeSpeed * inputs.modeinput;
  }

  public void moveOut() {
    io.moveOut(ffModel.calculate(IntakeConstants.kMaxOuttakeSpeed));
    intakeSpeed = IntakeConstants.kMaxOuttakeSpeed * inputs.modeinput;
  }

  public void stop() {
    stop = true;
    io.stop();
  }

  public double getVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
  }

  public double getVelocity() {
    return inputs.velocityRadPerSec;
  }

  public void runCharacterizationVolts(double volts) {
    io.setVoltage(volts);
  }

  public double getCharacterizationVelocity() {
    return inputs.velocityRadPerSec;
  }

  public boolean intakeEmpty() {
    return (inputs.velocityRadPerSec < IntakeConstants.threshold(intakeSpeed));
  }

  public boolean pieceHeld() {
    if(inputs.modeinput == conePiece){
      return (inputs.velocityRadPerSec < IntakeConstants.conePieceHeldThreshold);
    } else {
      return (inputs.velocityRadPerSec < IntakeConstants.cubePieceHeldThreshold);
    }
  }

  public void holdObject() {
    io.setVoltage(IntakeConstants.kHoldSpeed * 12.000 * inputs.modeinput); 
  }

  public void setMode(int m) {
    io.setMode(m);
  }

  public void toggleMode() {
    io.toggleMode();
  }

  public int getMode() {
    return inputs.modeinput;
  }
}
