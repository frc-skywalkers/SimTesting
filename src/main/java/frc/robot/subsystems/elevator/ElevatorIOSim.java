package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.ElevatorSimConstants;;

public class ElevatorIOSim implements ElevatorIO {
  private DCMotorSim sim = new DCMotorSim(DCMotor.getNEO(1), ElevatorSimConstants.gearing, ElevatorSimConstants.jKgMetersSquared);
  private PIDController pid = new PIDController(ElevatorSimConstants.kP, ElevatorSimConstants.kI, ElevatorSimConstants.kD);
  LinearFilter homingMovingAvg = LinearFilter.movingAverage(8);

  private boolean closedLoop = false;
  private double ffVolts = 0.0;
  private double appliedVolts = 0.0;
  public boolean isZeroed = false;

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    if (closedLoop) {
      appliedVolts =
          MathUtil.clamp(pid.calculate(sim.getAngularVelocityRadPerSec()) + ffVolts, -ElevatorSimConstants.maxVolts, ElevatorSimConstants.maxVolts);
      sim.setInputVoltage(appliedVolts);
    }

    sim.update(0.02);

    inputs.positionRad = 0.0;
    inputs.velocityRadPerSec = sim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = new double[] {sim.getCurrentDrawAmps()};
  }

  @Override
  public void setVoltage(double volts) {
    closedLoop = false;
    appliedVolts = 0.0;
    sim.setInputVoltage(volts);
  }

  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    closedLoop = true;
    pid.setSetpoint(velocityRadPerSec);
    this.ffVolts = ffVolts;
  }

  @Override
  public void stop() {
    setVoltage(0.0);
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pid.setPID(kP, kI, kD);
  }
}