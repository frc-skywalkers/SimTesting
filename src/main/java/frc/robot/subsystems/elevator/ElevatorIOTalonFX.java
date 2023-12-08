package frc.robot.subsystems.elevator;

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
import frc.robot.Constants.ElevatorConstants;

public class ElevatorIOTalonFX implements ElevatorIO {
  private final TalonFX leftElevator = new TalonFX(ElevatorConstants.kLeftElevatorPort);
  private final TalonFX rightElevator = new TalonFX(ElevatorConstants.kRightElevatorPort);

  private final StatusSignal<Double> Position = leftElevator.getPosition(); //for phoenix6
  private final StatusSignal<Double> Velocity = leftElevator.getVelocity();
  private final StatusSignal<Double> AppliedVolts = leftElevator.getMotorVoltage();
  private final StatusSignal<Double> Current = leftElevator.getStatorCurrent();
  private final StatusSignal<Double> followerCurrent = rightElevator.getStatorCurrent();

  public boolean isZeroed = false;

  public ElevatorIOTalonFX() {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = 30.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake; //i think sim doesnt have brake/coast mode which is why it slides a ton
    leftElevator.getConfigurator().apply(config);
    rightElevator.getConfigurator().apply(config);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, Position, Velocity, AppliedVolts, Current, followerCurrent);
    leftElevator.optimizeBusUtilization();
    rightElevator.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    BaseStatusSignal.refreshAll( //updates all inputs
        Position, Velocity, AppliedVolts, Current, followerCurrent);
    inputs.positionRad = Units.rotationsToRadians(Position.getValueAsDouble()) / ElevatorConstants.gearRatio; //puts them in autologged list for use in elevator.java
    inputs.velocityRadPerSec =
        Units.rotationsToRadians(Velocity.getValueAsDouble()) / ElevatorConstants.gearRatio;
    inputs.appliedVolts = AppliedVolts.getValueAsDouble();
    inputs.currentAmps =
        new double[] {Current.getValueAsDouble(), followerCurrent.getValueAsDouble()};
  }

  @Override
  public void setVoltage(double volts) {
    if (isZeroed) { //have to press reset button first or it wont run
      volts = MathUtil.clamp(volts, -ElevatorConstants.kMaxVolts, ElevatorConstants.kMaxVolts);
      leftElevator.setControl(new VoltageOut(volts));
    } else {
      stop();
    }
  }

  @Override
  public void setVelocity(double velocityRadPerSec) { //hm
    leftElevator.set(velocityRadPerSec);
    rightElevator.set(velocityRadPerSec);
  }

  @Override
  public void stop() {
    leftElevator.stopMotor();
    rightElevator.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    var config = new Slot0Configs();
    config.kP = kP;
    config.kI = kI;
    config.kD = kD;
    leftElevator.getConfigurator().apply(config);
    rightElevator.getConfigurator().apply(config);
  }

  public void getPosition() {
    //Elevator.elevatorposition = Position.getValueAsDouble();
  }

  public void reset() {
    leftElevator.setPosition(0);
    isZeroed = true;
  }

  public boolean zeroed() {return isZeroed;}
}