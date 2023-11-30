package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorIOSim implements ElevatorIO {
  //private ElevatorSim elevator = new ElevatorSim(DCMotor.getNEO(1), ElevatorConstants.gearRatio, ElevatorConstants.carraigeMassKg, ElevatorConstants.drumRadiusMeters, ElevatorConstants.minHeight, ElevatorConstants.maxHeight, true, 0);
  //uses basic trig to factor in angle between gravity and elevator
  private TiltedElevatorSim elevator = new TiltedElevatorSim(DCMotor.getFalcon500(1), ElevatorConstants.gearRatio, ElevatorConstants.carraigeMassKg, ElevatorConstants.drumRadiusMeters, ElevatorConstants.minHeight, ElevatorConstants.maxHeight, true);
  //maybe minheight should be 0.05 or something
  private PIDController pid = new PIDController(0.0, 0.0, 0.0);

  private boolean closedLoop = false;
  private double ffVolts = 0.0;
  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    elevator.update(0.02);
    //System.out.println("chchchhh"); //running
    inputs.positionRad = elevator.getPositionMeters();
    inputs.velocityRadPerSec = elevator.getVelocityMetersPerSecond(); //change name
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = new double[] {elevator.getCurrentDrawAmps()};
  }

  @Override
  public void setVoltage(double volts) {
    closedLoop = false;
    volts = MathUtil.clamp(volts, -ElevatorConstants.kMaxVoltsSim, ElevatorConstants.kMaxVoltsSim);
    appliedVolts = volts;
    elevator.setInputVoltage(volts);
  }

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