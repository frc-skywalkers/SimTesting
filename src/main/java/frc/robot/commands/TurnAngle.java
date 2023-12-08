// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Dashboard;
import frc.robot.subsystems.drive.Drive;


public class TurnAngle extends Command {
  /** Creates a new DriveForwardDistance. */
  private final PIDController rcontroller = new PIDController(0.15, 0, 0);

  private final double targetR; 

  Drive swerveSubsystem;

  boolean atSetpoint;

  double rspeed;
  double currentR;

  public TurnAngle(Drive swerveSubsystem, double targetR) { //meters, meters, degrees
    this.targetR = targetR;

    this.swerveSubsystem = swerveSubsystem;
    addRequirements(swerveSubsystem);

    rcontroller.setTolerance(LimelightConstants.rtolerance);
    rcontroller.enableContinuousInput(-180, 180);
  }

  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {
    currentR = swerveSubsystem.getYaw().getDegrees(); //

    if (currentR > 180) {
      currentR -= 360;
    }

    if (currentR < -180) {
      currentR += 360;
    }

    rspeed = 0.5 * MathUtil.clamp((rcontroller.calculate(currentR, targetR)), -LimelightConstants.rclamp, LimelightConstants.rclamp);
    
    rspeed += 0.5 * Math.signum(rspeed);

    if (rcontroller.atSetpoint()){
      rspeed = 0;
    }
    
    Dashboard.Limelight.Debugging.putNumber("rspeed", rspeed);

    atSetpoint = rcontroller.atSetpoint();

    swerveSubsystem.runVelocity(new ChassisSpeeds(0, 0, rspeed)); //should not be field oriented
  }

  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return ((Math.abs(targetR - currentR) < LimelightConstants.rtolerance) || (rspeed == 0));
  }
}