package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;

public class ArmIOSim implements ArmIO {
  //private ElevatorSim elevator = new ElevatorSim(DCMotor.getNEO(1), ElevatorConstants.gearRatio, ElevatorConstants.carraigeMassKg, ElevatorConstants.drumRadiusMeters, ElevatorConstants.minHeight, ElevatorConstants.maxHeight, true, 0);
  private SingleJointedArmSim arm = new SingleJointedArmSim(DCMotor.getNEO(1), ArmConstants.gearRatio, ArmConstants.jKgMetersSquared, ArmConstants.armLengthMeters, ArmConstants.minAngleRads, ArmConstants.maxAngleRads, true, ArmConstants.minAngleRads); //need to change starting position too
  private PIDController pid = new PIDController(0.0, 0.0, 0.0);

  private boolean closedLoop = false;
  private double ffVolts = 0.0;
  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    arm.update(0.02);
    inputs.positionRad = arm.getAngleRads();
    //inputs.velocityRadPerSec = leftElevator.getAngularVelocityRadPerSec();
    inputs.velocityRadPerSec = arm.getVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = new double[] {arm.getCurrentDrawAmps()};
  }

  @Override
  public void setVoltage(double volts) {
    closedLoop = false;
    volts = MathUtil.clamp(volts, -ArmConstants.kMaxVoltsSim, ArmConstants.kMaxVoltsSim);
    appliedVolts = volts;
    //leftElevator.setInputVoltage(volts);
    //rightElevator.setInputVoltage(volts);
    arm.setInputVoltage(volts);
  }

  /*
  @Override
  public void setVelocity(double velocityRadPerSec) {
    closedLoop = true;
    pid.setSetpoint(velocityRadPerSec);
  }
  */

  @Override
  public void stop() {
    setVoltage(0.0);
  }

  public void reset() {}

  /*
  @Override
  public void configurePID(double kP, double kI, double kD) {
    pid.setPID(kP, kI, kD);
  }
  */
}