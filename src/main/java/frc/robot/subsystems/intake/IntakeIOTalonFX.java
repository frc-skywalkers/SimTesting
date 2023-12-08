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

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.IntakeConstants;

public class IntakeIOTalonFX implements IntakeIO {
  private final TalonFX intakeMotor = new TalonFX(IntakeConstants.kIntakePort);

  private final StatusSignal<Double> Velocity = intakeMotor.getVelocity();
  private final StatusSignal<Double> AppliedVolts = intakeMotor.getMotorVoltage();
  private final StatusSignal<Double> Current = intakeMotor.getStatorCurrent();

  public final static int conePiece = -1; //not sure
  public final static int cubePiece = 1;
  public static int mode;

  private final boolean differentialIntake = IntakeConstants.differentialIntake;
  
  private double intakeSpeed = 0;
  public boolean stop = false; 


  public IntakeIOTalonFX() {
    
    var config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = 30.0; //
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake; //?
    intakeMotor.getConfigurator().apply(config);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, Velocity, AppliedVolts, Current);
    intakeMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        Velocity, AppliedVolts, Current);
    inputs.velocityRadPerSec =
        Units.rotationsToRadians(Velocity.getValueAsDouble()); //divide by gear ratio which is??
    inputs.appliedVolts = AppliedVolts.getValueAsDouble();
    inputs.currentAmps =
        new double[] {Current.getValueAsDouble()};
    inputs.modeinput = mode;
  }

  @Override
  public void setVoltage(double volts) {
    intakeMotor.setVoltage(volts); //hm
  }

  public void setSpeed(double speed) {
    intakeSpeed = speed;
    intakeMotor.set(intakeSpeed);
    SmartDashboard.putNumber("Intake Set Speed", speed);
  }

  public void moveIn(double ffvolts) {
    setSpeed(IntakeConstants.kMaxIntakeSpeed * mode);
    intakeSpeed = IntakeConstants.kMaxIntakeSpeed * mode;
  }

  public void moveOut(double ffvolts) {
    setSpeed(IntakeConstants.kMaxOuttakeSpeed * mode);
    intakeSpeed = IntakeConstants.kMaxOuttakeSpeed * mode;
  }

  @Override
  public void stop() {
    stop = true;
        setSpeed(0.000);
        intakeSpeed = 0;
  }

  public double getActualVelocity() {
    return Math.abs(Velocity.getValueAsDouble());
  }

  public double getActualCurrent() {
    return Current.getValueAsDouble();
  }

  public boolean intakeEmpty() {
    return getActualVelocity() > IntakeConstants.threshold(intakeSpeed);
  }

  public boolean pieceHeld() {
    if(mode == conePiece) {
      return getActualCurrent() > IntakeConstants.conePieceHeldThreshold;
    } else {
      return getActualCurrent() > IntakeConstants.cubePieceHeldThreshold;
    }
  }

  private double getThreshold() {
    if(mode == conePiece) {
      return IntakeConstants.conePieceHeldThreshold;
    } else {
      return IntakeConstants.cubePieceHeldThreshold;
    }
  }

  public void setMode(int m) {
    this.mode = m;
  }

  public int getMode() {
    return mode;
  }

  public void toggleMode() {
    if (mode == conePiece) {
      setMode(cubePiece);
    } else if (mode == cubePiece) {
      setMode(conePiece);
    }
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    var config = new Slot0Configs();
    config.kP = kP;
    config.kI = kI;
    config.kD = kD;
    intakeMotor.getConfigurator().apply(config);
  }
}