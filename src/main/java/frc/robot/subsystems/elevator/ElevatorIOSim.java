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
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorSimConstants;

//DOES NOT WORK DOES NOT WORK DOES NOT WORK - unless?
public class ElevatorIOSim implements ElevatorIO {
  private DCMotorSim leftElevator = new DCMotorSim(DCMotor.getNEO(1), ElevatorSimConstants.gearing, ElevatorSimConstants.jKgMetersSquared);
  private DCMotorSim rightElevator = new DCMotorSim(DCMotor.getNEO(1), ElevatorSimConstants.gearing, ElevatorSimConstants.jKgMetersSquared);


  public boolean isZeroed = true;
  public boolean softLimitsEnabled = false;
  public double goal;
  public boolean m_enabled = false;

  private final ProfiledPIDController profiledPIDController = new ProfiledPIDController(
        ElevatorConstants.kP,
        0.0,
        0.0,
        new TrapezoidProfile.Constraints(ElevatorConstants.kMaxVel, ElevatorConstants.kMaxAcc));;

  public ElevatorIOSim() {
    profiledPIDController.setTolerance(0.02); 
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) { //goes
    inputs.positionRad = leftElevator.getAngularPositionRad();
    inputs.velocityRadPerSec = leftElevator.getAngularVelocityRPM();
    inputs.currentAmps =
        new double[] {leftElevator.getCurrentDrawAmps(), rightElevator.getCurrentDrawAmps()};
  }

  public void setVoltage(double volts) { //yes
    if (isZeroed) {
        System.out.println("jjj");
        volts = MathUtil.clamp(volts, -8, 8);
        rightElevator.setInputVoltage(volts);
        leftElevator.setInputVoltage(volts);
    } else {
        stop();
    }
  }

  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    double feedforward = 0;
    if (setpoint.velocity > 0) {
      feedforward = setpoint.velocity * ElevatorConstants.kVUp + ElevatorConstants.kSUp;
    } else {
      System.out.println(":))))");
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
  }

  public boolean atGoal() {
    // Dashboard.Elevator.Driver.putBoolean("Elevator Goal Reached", this.getController().atGoal());
    return Math.abs(getPosition() - goal) <= 0.07;
  }

  public void setGoal(double position) {
    goal = position;
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
    return leftElevator.getAngularPositionRad() * ElevatorConstants.kPositionConversionFactor;
  }

  public double getVelocity() {
    return leftElevator.getAngularVelocityRadPerSec() * ElevatorConstants.kVelocityConversionFactor;
  }

  public double getCurrent() {
    return leftElevator.getCurrentDrawAmps();
  }

  public void stop() {
    leftElevator.setInputVoltage(0);
  }

  public void resetEncoders() {
    leftElevator.setState(0, 0);
    rightElevator.setState(0, 0);
  }

  public void periodic() {
    if (m_enabled) {
      System.out.println("wjakjsdkjfklaj");
      useOutput(profiledPIDController.calculate(leftElevator.getAngularPositionRad()), profiledPIDController.getSetpoint());
    }
    if (atGoal()) {
      m_enabled = false;
    }
  }

  public void configurePID(double kP, double kI, double kD) {
    var config = new Slot0Configs();
    config.kP = kP;
    config.kI = kI;
    config.kD = kD;
    //eftElevator.getConfigurator().apply(config);
  }


}