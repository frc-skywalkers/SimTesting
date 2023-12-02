// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;

public class Visualizer extends SubsystemBase {
  /** Creates a new Visualizer. */

  Arm arm;
  Elevator elevator;
  Mechanism2d mech;

  private MechanismLigament2d m_elevator;
  private MechanismLigament2d m_arm;
  private MechanismLigament2d m_joint;

  MechanismRoot2d root;


  public Visualizer(Arm arm, Elevator elevator) {
    this.arm = arm;
    this.elevator = elevator;
    mech = new Mechanism2d(3, 3);
    root = mech.getRoot("angled elev", 1, 0.5);

    m_elevator = root.append(new MechanismLigament2d("elevator", 0.15, 55));
    m_joint =
        m_elevator.append(
            new MechanismLigament2d("joint", 0.3, -55, 1, new Color8Bit(Color.kPurple)));
    m_arm =
      m_joint.append(
          new MechanismLigament2d("arm", 0.15, 0, 3, new Color8Bit(Color.kPink)));
  
    SmartDashboard.putData("Mech2d", mech);
    Logger.recordOutput("My Mechanism", mech);
  }

  public void setElevatorLength(double length) {
    m_elevator.setLength(0.15 + length);
  }
  
  public void setArmAngle(double angle) {
    m_arm.setAngle(angle * 360 / 2 / Math.PI);
  }

  public Mechanism2d getMech() {
    return mech;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // setElevatorLength(elevator.getPosition());
    // setArmAngle(arm.getPosition());
    // Logger.recordOutput("observed elev length", elevator.getPosition());
    // Logger.recordOutput("observed arm angle", arm.getPosition());
    // Logger.recordOutput("My Mechanism", mech);
  }
}
