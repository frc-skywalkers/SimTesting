package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.TalonFXModuleConstants;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn motor controller, and
 * CANcoder
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations (e.g. If using an analog encoder, copy from "ModuleIOSparkMax")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOTalonFX implements ModuleIO {
  private final TalonFX driveTalon;
  private final TalonFX turnTalon;
  private final CANcoder cancoder;

  private final StatusSignal<Double> drivePosition;
  private final StatusSignal<Double> driveVelocity;
  private final StatusSignal<Double> driveAppliedVolts;
  private final StatusSignal<Double> driveCurrent;

  private final StatusSignal<Double> turnAbsolutePosition;
  private final StatusSignal<Double> turnPosition;
  private final StatusSignal<Double> turnVelocity;
  private final StatusSignal<Double> turnAppliedVolts;
  private final StatusSignal<Double> turnCurrent;

  // Gear ratios for SDS MK4i L2, adjust as necessary
  private final Rotation2d absoluteEncoderOffset;

  public ModuleIOTalonFX(int index) {
    
    switch (index) { //ick
      case 0:
        driveTalon = new TalonFX(TalonFXModuleConstants.Mod0.driveMotorID, "CANivore");
        driveTalon.setInverted(TalonFXModuleConstants.driveEncoderInverts[0]);
        turnTalon = new TalonFX(TalonFXModuleConstants.Mod0.angleMotorID, "CANivore");
        turnTalon.setInverted(TalonFXModuleConstants.turnEncoderInverts[0]);
        cancoder = new CANcoder(TalonFXModuleConstants.Mod0.canCoderID, "CANivore");
        absoluteEncoderOffset = TalonFXModuleConstants.Mod0.angleOffset;
        break;
      case 1:
        driveTalon = new TalonFX(TalonFXModuleConstants.Mod1.driveMotorID, "CANivore");
        driveTalon.setInverted(TalonFXModuleConstants.driveEncoderInverts[1]);
        turnTalon = new TalonFX(TalonFXModuleConstants.Mod1.angleMotorID, "CANivore");
        turnTalon.setInverted(TalonFXModuleConstants.turnEncoderInverts[1]);
        cancoder = new CANcoder(TalonFXModuleConstants.Mod1.canCoderID, "CANivore");
        absoluteEncoderOffset = TalonFXModuleConstants.Mod1.angleOffset; 
        break;
      case 2:
        driveTalon = new TalonFX(TalonFXModuleConstants.Mod2.driveMotorID, "CANivore");
        driveTalon.setInverted(TalonFXModuleConstants.driveEncoderInverts[2]);
        turnTalon = new TalonFX(TalonFXModuleConstants.Mod2.angleMotorID, "CANivore");
        turnTalon.setInverted(TalonFXModuleConstants.turnEncoderInverts[2]);
        cancoder = new CANcoder(TalonFXModuleConstants.Mod2.canCoderID, "CANivore");
        absoluteEncoderOffset = TalonFXModuleConstants.Mod2.angleOffset; 
        break;
      case 3:
        driveTalon = new TalonFX(TalonFXModuleConstants.Mod3.driveMotorID, "CANivore");
        driveTalon.setInverted(TalonFXModuleConstants.driveEncoderInverts[3]);
        turnTalon = new TalonFX(TalonFXModuleConstants.Mod3.angleMotorID, "CANivore");
        turnTalon.setInverted(TalonFXModuleConstants.turnEncoderInverts[3]);
        cancoder = new CANcoder(TalonFXModuleConstants.Mod3.canCoderID, "CANivore");
        absoluteEncoderOffset = TalonFXModuleConstants.Mod3.angleOffset; 
        break;
      default:
        throw new RuntimeException("Invalid module index");
    }

    var driveConfig = new TalonFXConfiguration();
    driveConfig.CurrentLimits.StatorCurrentLimit = TalonFXModuleConstants.DriveStatorCurrentLimit;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveTalon.getConfigurator().apply(driveConfig);
    setDriveBrakeMode(TalonFXModuleConstants.driveBrakeMode);

    var turnConfig = new TalonFXConfiguration();
    turnConfig.CurrentLimits.StatorCurrentLimit = TalonFXModuleConstants.TurnStatorCurrentLimit;
    turnConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    turnTalon.getConfigurator().apply(turnConfig);
    setTurnBrakeMode(TalonFXModuleConstants.turnBrakeMode);

    cancoder.getConfigurator().apply(new CANcoderConfiguration());

    drivePosition = driveTalon.getPosition();
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveCurrent = driveTalon.getStatorCurrent();

    turnAbsolutePosition = cancoder.getAbsolutePosition();
    turnPosition = turnTalon.getPosition();
    turnVelocity = turnTalon.getVelocity();
    turnAppliedVolts = turnTalon.getMotorVoltage();
    turnCurrent = turnTalon.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        TalonFXModuleConstants.odometryUpdateFreqHz, drivePosition, turnPosition); // Required for odometry, use faster rate
    BaseStatusSignal.setUpdateFrequencyForAll(
        TalonFXModuleConstants.otherUpdateFreqHz,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        turnAbsolutePosition,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent);
    driveTalon.optimizeBusUtilization();
    turnTalon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        drivePosition,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        turnAbsolutePosition,
        turnPosition,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent);

    inputs.drivePositionRad =
        Units.rotationsToRadians(drivePosition.getValueAsDouble()) / TalonFXModuleConstants.driveGearRatio;
    inputs.driveVelocityRadPerSec =
        Units.rotationsToRadians(driveVelocity.getValueAsDouble()) / TalonFXModuleConstants.driveGearRatio;
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = new double[] {driveCurrent.getValueAsDouble()};

    inputs.turnAbsolutePosition =
        Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble())
            .minus(absoluteEncoderOffset);
    inputs.turnPosition =
        Rotation2d.fromRotations(turnPosition.getValueAsDouble() / TalonFXModuleConstants.turnGearRatio);
    inputs.turnVelocityRadPerSec =
        Units.rotationsToRadians(turnVelocity.getValueAsDouble()) / TalonFXModuleConstants.turnGearRatio;
    inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
    inputs.turnCurrentAmps = new double[] {turnCurrent.getValueAsDouble()};
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveTalon.setControl(new VoltageOut(volts));
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnTalon.setControl(new VoltageOut(volts));
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    var config = new MotorOutputConfigs();
    config.Inverted = InvertedValue.CounterClockwise_Positive;
    config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    driveTalon.getConfigurator().apply(config);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    var config = new MotorOutputConfigs();
    config.Inverted =
        TalonFXModuleConstants.turnMotorInvert
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    turnTalon.getConfigurator().apply(config);
  }
}
