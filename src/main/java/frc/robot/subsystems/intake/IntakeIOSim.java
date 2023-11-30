// Copyright 2021-2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.IntakeConstants;

public class IntakeIOSim implements IntakeIO {
  private DCMotorSim intakeSim = new DCMotorSim(DCMotor.getFalcon500(1), 1.5, 0.004);
  private PIDController pid = new PIDController(0.0, 0.0, 0.0);

  private boolean closedLoop = false;
  private double ffVolts = 0.0;
  private double appliedVolts = 0.0;

  public final static int conePiece = 1;
  public final static int cubePiece = -1;

  public Mode mode = Mode.CUBE;

  public enum Mode {
    CONE(1),
    CUBE(-1);

    public int multiplier;

    Mode(int m) {
      multiplier = m;
    }
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    if (closedLoop) {
      appliedVolts =
          MathUtil.clamp(pid.calculate(intakeSim.getAngularVelocityRadPerSec()) + ffVolts, -12.0, 12.0); //pid+feedforward, clamped
      intakeSim.setInputVoltage(appliedVolts);
    }

    intakeSim.update(0.02);

    inputs.positionRad = 0.0;
    inputs.velocityRadPerSec = intakeSim.getAngularVelocityRadPerSec(); //takes updated values and puts them in autologged for intake.java
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = new double[] {intakeSim.getCurrentDrawAmps()};
  }

  @Override
  public void setVoltage(double volts) {
    closedLoop = false;
    appliedVolts = 0.0;
    intakeSim.setInputVoltage(volts);
  }

  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    closedLoop = true;
    pid.setSetpoint(velocityRadPerSec);
    this.ffVolts = ffVolts;
  }

  public void moveIn(double ffVolts) {
    setVelocity(IntakeConstants.kMaxIntakeSpeed * mode.multiplier, ffVolts);
  }

  public void moveOut(double ffVolts) {
    setVelocity(IntakeConstants.kMaxOuttakeSpeed * mode.multiplier, ffVolts);
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
