package frc.robot.subsystems.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.ArmConstants;

public class ArmIOTalonFX implements ArmIO {
  private final TalonFX armMotor = new TalonFX(ArmConstants.kArmPort);

  private final StatusSignal<Double> Position = armMotor.getPosition();
  private final StatusSignal<Double> Velocity = armMotor.getVelocity(); //rotations per second
  private final StatusSignal<Double> AppliedVolts = armMotor.getMotorVoltage();
  private final StatusSignal<Double> Current = armMotor.getStatorCurrent();
  private final StatusSignal<Double> followerCurrent = armMotor.getStatorCurrent();

  public boolean isZeroed = false;

  public ArmIOTalonFX() {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = ArmConstants.StatorCurrentLimit; //not sure
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    armMotor.getConfigurator().apply(config);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, Position, Velocity, AppliedVolts, Current, followerCurrent);
    armMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        Position, Velocity, AppliedVolts, Current, followerCurrent);
    inputs.positionRad = Units.rotationsToRadians(Position.getValueAsDouble()) / ArmConstants.gearRatio;
    inputs.velocityRadPerSec =
        Units.rotationsToRadians(Velocity.getValueAsDouble()) / ArmConstants.gearRatio;
    inputs.appliedVolts = AppliedVolts.getValueAsDouble();
    inputs.currentAmps =
        new double[] {Current.getValueAsDouble(), followerCurrent.getValueAsDouble()};
  }

  @Override
  public void setVoltage(double volts) {
    volts = MathUtil.clamp(volts, -ArmConstants.kMaxVolts, ArmConstants.kMaxVolts);
    armMotor.setControl(new VoltageOut(volts));
  }

  @Override
  public void setVelocity(double velocityRadPerSec) { //hm
    armMotor.set(velocityRadPerSec);
  }

  @Override
  public void stop() {
    armMotor.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    var config = new Slot0Configs();
    config.kP = kP;
    config.kI = kI;
    config.kD = kD;
    armMotor.getConfigurator().apply(config);
  }

  public void getPosition() {
    //Elevator.elevatorposition = Position.getValueAsDouble();
  }

  public void reset() { //we dont need this?
    armMotor.setPosition(0);
  }
}