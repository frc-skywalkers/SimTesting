package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ElevatorConstants;

//DOES NOT WORK DOES NOT WORK DOES NOT WORK
public class ElevatorIOTalonFX implements ElevatorIO {
  private final TalonFX leftElevator = new TalonFX(ElevatorConstants.kLeftElevatorPort);
  private final TalonFX rightElevator = new TalonFX(ElevatorConstants.kRightElevatorPort);
  LinearFilter homingMovingAvg = LinearFilter.movingAverage(8);

  public boolean isZeroed = false;
  public boolean softLimitsEnabled = false;

  private final StatusSignal<Double> Position = leftElevator.getPosition();
  private final StatusSignal<Double> Velocity = leftElevator.getVelocity();
  private final StatusSignal<Double> AppliedVolts = leftElevator.getMotorVoltage();
  private final StatusSignal<Double> LeftCurrent = leftElevator.getStatorCurrent();
  private final StatusSignal<Double> RightCurrent = rightElevator.getStatorCurrent();

  public ElevatorIOTalonFX() {
    ProfiledPIDController profiledPIDController = new ProfiledPIDController(
        ElevatorConstants.kP,
        0.0,
        0.0,
        new TrapezoidProfile.Constraints(ElevatorConstants.kMaxVel, ElevatorConstants.kMaxAcc));
    profiledPIDController.setTolerance(0.02); 

    var config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = ElevatorConstants.StatorCurrentLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    leftElevator.getConfigurator().apply(config);
    rightElevator.getConfigurator().apply(config);
    leftElevator.setInverted(ElevatorConstants.kLeftInverted);
    rightElevator.setInverted(ElevatorConstants.kRightInverted);
    //missing feedbackdevice integratedsensor, phoenix6 imports?
    //config soft limits
    //reset encoders

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, Position, Velocity, AppliedVolts, LeftCurrent, RightCurrent);

    leftElevator.optimizeBusUtilization();
    rightElevator.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        Position, Velocity, AppliedVolts, LeftCurrent, RightCurrent);
    inputs.positionRad = Units.rotationsToRadians(Position.getValueAsDouble()) / ElevatorConstants.gearRatio;
    inputs.velocityRadPerSec =
        Units.rotationsToRadians(Velocity.getValueAsDouble()) / ElevatorConstants.gearRatio;
    inputs.appliedVolts = AppliedVolts.getValueAsDouble();
    inputs.currentAmps =
        new double[] {LeftCurrent.getValueAsDouble(), RightCurrent.getValueAsDouble()};
  }

  public void setVoltage(double volts) {
    if (isZeroed) {
        volts = MathUtil.clamp(volts, -8, 8);
        rightElevator.setControl(new VoltageOut(volts));
        leftElevator.setControl(new VoltageOut(volts));
    } else {
        stop();
    }
  }

  public void setVelocity(double speed) { //double ffvolts
    speed = MathUtil.clamp(speed, -ElevatorConstants.kMaxElevatorSpeed, ElevatorConstants.kMaxElevatorSpeed);
    rightElevator.set(speed);
    leftElevator.set(speed);
    //softlimits
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

  public double getPosition() {
    return Position.getValueAsDouble() * ElevatorConstants.kPositionConversionFactor;
  }

  public double getVelocity() {
    return Velocity.getValueAsDouble() * ElevatorConstants.kVelocityConversionFactor;
  }

  public double getCurrent() {
    return homingMovingAvg.calculate((LeftCurrent.getValueAsDouble() + RightCurrent.getValueAsDouble()) / 2.0);
  }

  public void stop() {
    leftElevator.stopMotor();
  }

  public void resetEncoders() {
    //
  }

  public void configurePID(double kP, double kI, double kD) {
    var config = new Slot0Configs();
    config.kP = kP;
    config.kI = kI;
    config.kD = kD;
    leftElevator.getConfigurator().apply(config);
  }


}