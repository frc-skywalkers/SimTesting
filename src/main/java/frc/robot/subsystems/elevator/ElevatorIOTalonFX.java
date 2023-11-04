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

//DOES NOT WORK DOES NOT WORK DOES NOT WORK - unless?
public class ElevatorIOTalonFX implements ElevatorIO {
  private final TalonFX leftElevator = new TalonFX(ElevatorConstants.kLeftElevatorPort);
  private final TalonFX rightElevator = new TalonFX(ElevatorConstants.kRightElevatorPort);

  public boolean isZeroed = false;
  public boolean softLimitsEnabled = false;
  public double goal;
  public boolean m_enabled = false;

  private final StatusSignal<Double> Position = leftElevator.getPosition();
  private final StatusSignal<Double> Velocity = leftElevator.getVelocity();
  private final StatusSignal<Double> AppliedVolts = leftElevator.getMotorVoltage();
  private final StatusSignal<Double> LeftCurrent = leftElevator.getStatorCurrent();
  private final StatusSignal<Double> RightCurrent = rightElevator.getStatorCurrent();
  private final ProfiledPIDController profiledPIDController = new ProfiledPIDController(
        ElevatorConstants.kP,
        0.0,
        0.0,
        new TrapezoidProfile.Constraints(ElevatorConstants.kMaxVel, ElevatorConstants.kMaxAcc));;

  public ElevatorIOTalonFX() {
    profiledPIDController.setTolerance(0.02); 

    TalonFXConfiguration leftconfig = new TalonFXConfiguration();
    leftconfig.CurrentLimits.StatorCurrentLimit = ElevatorConstants.StatorCurrentLimit;
    leftconfig.CurrentLimits.StatorCurrentLimitEnable = true;
    leftconfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    leftconfig.Feedback.FeedbackRemoteSensorID = ElevatorConstants.kLeftElevatorPort;
    TalonFXConfiguration rightconfig = new TalonFXConfiguration();
    rightconfig.CurrentLimits.StatorCurrentLimit = ElevatorConstants.StatorCurrentLimit;
    rightconfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rightconfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rightconfig.Feedback.FeedbackRemoteSensorID = ElevatorConstants.kLeftElevatorPort; 
    leftElevator.getConfigurator().apply(leftconfig);
    rightElevator.getConfigurator().apply(rightconfig);

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

  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    double feedforward = 0;
    if (setpoint.velocity > 0) {
      feedforward = setpoint.velocity * ElevatorConstants.kVUp + ElevatorConstants.kSUp;
    } else {
      feedforward = setpoint.velocity * ElevatorConstants.kVDown + ElevatorConstants.kSDown;
    }

    if (isZeroed) {
      setVoltage(feedforward + output);
    } else {
      // System.out.println("ELEVATOR NOT ZEROED!");
    }
  }

  
  public void goToPosition(double position) {
    goal = position;
    m_enabled = true;
    if (atGoal()){
      m_enabled = false;
    }
  }

  public boolean atGoal() {
    // Dashboard.Elevator.Driver.putBoolean("Elevator Goal Reached", this.getController().atGoal());
    return Math.abs(getPosition() - goal) <= 0.07;
  }

  public void setGoal(double position) {
    goal = position;
  }

  /* 
  public void setVelocity(double speed) { //double ffvolts
    speed = MathUtil.clamp(speed, -ElevatorConstants.kMaxElevatorSpeed, ElevatorConstants.kMaxElevatorSpeed);
    rightElevator.set(speed);
    leftElevator.set(speed);
    //softlimits
  }
  */

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
    return LeftCurrent.getValueAsDouble();
  }

  public void stop() {
    leftElevator.stopMotor();
  }

  public void resetEncoders() {
    leftElevator.getConfigurator().setPosition(0);
    rightElevator.getConfigurator().setPosition(0);
  }

  public void periodic() {
    if (m_enabled) {
      useOutput(profiledPIDController.calculate(leftElevator.getPosition().getValueAsDouble()), new TrapezoidProfile.State(goal, 0));
    }
  }

  public void configurePID(double kP, double kI, double kD) {
    var config = new Slot0Configs();
    config.kP = kP;
    config.kI = kI;
    config.kD = kD;
    leftElevator.getConfigurator().apply(config);
  }


}