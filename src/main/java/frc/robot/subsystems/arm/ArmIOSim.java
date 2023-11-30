package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;

public class ArmIOSim implements ArmIO {
  //the last value is the starting position
  //not sure what jKgMetersSquared is
  private SingleJointedArmSim arm = new SingleJointedArmSim(DCMotor.getFalcon500(1), ArmConstants.gearRatio, ArmConstants.jKgMetersSquared, ArmConstants.armLengthMeters, ArmConstants.minAngleRads, ArmConstants.maxAngleRads, true, ArmConstants.minAngleRads);
  private PIDController pid = new PIDController(0.0, 0.0, 0.0); //gets configured based on constants in Arm.java

  private boolean closedLoop = false;
  private double ffVolts = 0.0;
  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(ArmIOInputs inputs) { //puts info from sim into autologged inputs
    arm.update(0.02);
    inputs.positionRad = arm.getAngleRads(); 
    inputs.velocityRadPerSec = arm.getVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = new double[] {arm.getCurrentDrawAmps()};
  }

  @Override
  public void setVoltage(double volts) {
    closedLoop = false;
    volts = MathUtil.clamp(volts, -ArmConstants.kMaxVoltsSim, ArmConstants.kMaxVoltsSim);
    appliedVolts = volts;
    arm.setInputVoltage(volts);
  }

  @Override
  public void stop() {
    setVoltage(0.0);
  }

  public void reset() {}

  //not using rn because of profiled pid
  /*
  @Override
  public void configurePID(double kP, double kI, double kD) {
    pid.setPID(kP, kI, kD);
  }
  */
}