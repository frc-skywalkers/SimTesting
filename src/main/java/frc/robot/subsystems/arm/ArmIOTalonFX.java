package frc.robot.subsystems.arm;

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
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;

//DOES NOT WORK DOES NOT WORK DOES NOT WORK
public class ArmIOTalonFX implements ArmIO {
  private final TalonFX arm = new TalonFX(ArmConstants.kArmPort);
  LinearFilter homingMovingAvg = LinearFilter.movingAverage(8);

  public boolean isZeroed = false;
  public boolean softLimitsEnabled = false;

  private final StatusSignal<Double> Position = arm.getPosition();
  private final StatusSignal<Double> Velocity = arm.getVelocity();
  private final StatusSignal<Double> AppliedVolts = arm.getMotorVoltage();
  private final StatusSignal<Double> Current = arm.getStatorCurrent();

  public ArmIOTalonFX() {
    ProfiledPIDController profiledPIDController = new ProfiledPIDController(
        ArmConstants.kP,
        ArmConstants.kI,
        ArmConstants.kD,
        new TrapezoidProfile.Constraints(3, 4));
    profiledPIDController.setTolerance(0.03); 

    var config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = ElevatorConstants.StatorCurrentLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    arm.getConfigurator().apply(config);
    arm.setInverted(ArmConstants.kInverted);
    //missing feedbackdevice integratedsensor, phoenix6 imports?
    //config soft limits
    //reset encoders

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, Position, Velocity, AppliedVolts, Current);

    arm.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        Position, Velocity, AppliedVolts, Current);
    inputs.positionRad = Units.rotationsToRadians(Position.getValueAsDouble()) / ArmConstants.gearRatio;
    inputs.velocityRadPerSec =
        Units.rotationsToRadians(Velocity.getValueAsDouble()) / ArmConstants.gearRatio;
    inputs.appliedVolts = AppliedVolts.getValueAsDouble();
    inputs.currentAmps =
        new double[] {Current.getValueAsDouble()};
  }

  public void setVoltage(double volts) {
    if (isZeroed) {
        volts = MathUtil.clamp(volts, -8, 8);
        arm.setControl(new VoltageOut(volts));
    } else {
        stop();
    }
  }

  public void setVelocity(double speed) { //double ffvolts
    speed = MathUtil.clamp(speed, -ElevatorConstants.kMaxElevatorSpeed, ElevatorConstants.kMaxElevatorSpeed);
    arm.set(speed);
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
    return Position.getValueAsDouble() * ArmConstants.kPositionConversionFactor;
  }

  public double getVelocity() {
    return Velocity.getValueAsDouble() * ArmConstants.kVelocityConversionFactor;
  }

  public void stop() {
    arm.stopMotor();
  }

  public void resetEncoders() {
    //
  }

  public void configurePID(double kP, double kI, double kD) {
    var config = new Slot0Configs();
    config.kP = kP;
    config.kI = kI;
    config.kD = kD;
    arm.getConfigurator().apply(config);
  }


}