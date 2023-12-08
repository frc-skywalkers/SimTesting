package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

import java.security.DrbgParameters.Reseed;

import edu.wpi.first.math.MathUtil;
import frc.robot.Dashboard;
import frc.robot.Constants.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.drive.Drive;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;


public class AlignCone extends Command {

  private final PIDController xcontroller = new PIDController(8, 0, 0); //all in degrees
  private final PIDController ycontroller = new PIDController(8, 0, 0);
  private final PIDController rcontroller = new PIDController(0.15, 0, 0);

  private final double targetXDist;
  private final double targetYDist;

  double xspeed;
  double yspeed;
  double minxspeed;
  double minyspeed;

  Drive swerveSubsystem;
  Limelight camera;

  boolean atSetpoint;
  boolean ydistreached;
  boolean xdistreached;

  double currentXdistance;
  double currentYdistance;

  public AlignCone(Drive swerveSubsystem, Limelight camera, double targetXDist, double targetYDist, double targetR) { //meters, meters, degrees
    this.targetXDist = targetXDist; //positive forward
    this.targetYDist = targetYDist; //positive when robot to the right

    this.swerveSubsystem = swerveSubsystem;
    this.camera = camera;
    addRequirements(swerveSubsystem);
    addRequirements(camera);
  }

  @Override
  public void initialize() {
    xcontroller.setTolerance(LimelightConstants.xtolerance);
    ycontroller.setTolerance(LimelightConstants.ytolerance);
    rcontroller.setTolerance(LimelightConstants.rtolerance);
    ydistreached = false;
    xdistreached = false;
    /* 
    if (swerveSubsystem.getFieldOriented()){
      swerveSubsystem.toggleField();
    }
    */
  }

  @Override
  public void execute() {
    double currentYAngle = camera.getRTTX(); //-, limelight and swerve directions swapped, ref frame (robot to the right +)
    double currentXAngle = camera.getRTTY(); 

    currentXdistance = (LimelightConstants.RTheight - LimelightConstants.cameraheight)/Math.tan(currentXAngle*Math.PI/180); //radians
    currentYdistance = Math.tan(currentYAngle*Math.PI/180) * currentXdistance; //+

    //double targetXAngle = Math.atan((LimelightConstants.RTheight - LimelightConstants.cameraheight)/targetXDist) - LimelightConstants.mountingangle; //upwards angle
    //double targetYAngle = Math.atan(((targetYDist - LimelightConstants.limelightOffsetCenter)/targetXDist)); //+-?

    double robottargetY = targetYDist + LimelightConstants.limelightOffsetCenter;

    xspeed = -1 * MathUtil.clamp((xcontroller.calculate(currentXdistance, targetXDist)), -LimelightConstants.xclamp, LimelightConstants.xclamp);
    yspeed = 1 * MathUtil.clamp((ycontroller.calculate(currentYdistance, robottargetY)), -LimelightConstants.yclamp, LimelightConstants.yclamp); //-


    if (xcontroller.atSetpoint()){
      xspeed = 0;
      xdistreached = true;
    }
    if (ycontroller.atSetpoint()){
      yspeed = 0;
      ydistreached = true;
    }

    if (Math.abs(xspeed) < 0.2){
      xspeed =0;
    }
    if (Math.abs(yspeed) < 0.2){
      yspeed = 0;
    }
    
    Dashboard.TeleDash.Debugging.putNumber("currentxdist", currentXdistance);
    Dashboard.TeleDash.Debugging.putNumber("currentydist", currentYdistance);

    Dashboard.TeleDash.Debugging.putNumber("xspeed", xspeed);
    Dashboard.TeleDash.Debugging.putNumber("yspeed", yspeed);

    //Dashboard.Tele.Debugging.putNumber("xerror", targetXAngle-currentXAngle);
    //Dashboard.Tele.Debugging.putNumber("yerror", targetYAngle-currentYAngle);
    
    Dashboard.TeleDash.Debugging.putNumber("xdistance", currentXdistance);
    Dashboard.TeleDash.Debugging.putNumber("ydistance", currentYdistance);


    atSetpoint = (xcontroller.atSetpoint() && ycontroller.atSetpoint() && rcontroller.atSetpoint());

    swerveSubsystem.runVelocity(new ChassisSpeeds(xspeed, yspeed, 0)); //should be NOT field oriented
  }

  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return ((Math.abs(targetXDist - currentXdistance) < LimelightConstants.xtolerance) ||
      (Math.abs(targetYDist - currentYdistance) < LimelightConstants.ytolerance) ||
      (xspeed == 0 && yspeed == 0));
  }
}