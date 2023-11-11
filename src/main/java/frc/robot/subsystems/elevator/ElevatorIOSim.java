package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {
  private ElevatorSim elevator = new ElevatorSim(DCMotor.getNEO(1), 1.5, 2.0, 0.5, 0, 4.5, true, 0.5);
  //maybe minheight should be 0.05 or something
  //private DCMotorSim leftElevator = new DCMotorSim(DCMotor.getNEO(1), 1.5, 0.004);
  //private DCMotorSim rightElevator = new DCMotorSim(DCMotor.getNEO(1), 1.5, 0.004);
  private PIDController pid = new PIDController(0.0, 0.0, 0.0);

  private boolean closedLoop = false;
  private double ffVolts = 0.0;
  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    elevator.update(0.02);
    //System.out.println("chchchhh"); //running
    //inputs.positionRad = leftElevator.getAngularPositionRad();
    //problem here
    inputs.positionRad = elevator.getPositionMeters(); //getPositionMeters problem
    //inputs.velocityRadPerSec = leftElevator.getAngularVelocityRadPerSec();
    inputs.velocityRadPerSec = elevator.getVelocityMetersPerSecond(); //change name
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = new double[] {elevator.getCurrentDrawAmps()};
  }

  @Override
  public void setVoltage(double volts) {
    closedLoop = false;
    volts = MathUtil.clamp(volts, -8, 8);
    appliedVolts = volts;
    //leftElevator.setInputVoltage(volts);
    //rightElevator.setInputVoltage(volts);
    elevator.setInputVoltage(volts);
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

  /*
  @Override
  public void configurePID(double kP, double kI, double kD) {
    pid.setPID(kP, kI, kD);
  }
  */
}