package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Mode currentMode = Mode.SIM;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public final class DriveConstants {
    public static final double DEADBAND = 0.15;
    public static final double MAX_LINEAR_SPEED = 5.00; //meters per second
    public static final double TRACK_WIDTH_X = Units.inchesToMeters(24.0);
    public static final double TRACK_WIDTH_Y = Units.inchesToMeters(24.0);
    public static final double DRIVE_BASE_RADIUS = Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
    public static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;

    public static final int PigeonID = 13;
  }

  public final class FFConstants {
    public static final double START_DELAY_SECS = 2.0;
    public static final double RAMP_VOLTS_PER_SEC = 0.1;
  }

  public final class ModuleConstants {
    public static final double realDrivekS = 0.634; //from skywalkers-offseason
    public static final double realDrivekV = 2.16;
    public static final double realDrivekP = 0.05;
    public static final double realDrivekI = 0;
    public static final double realDrivekD = 0;
    public static final double realTurnkP = 0.65;
    public static final double realTurnkI = 0;
    public static final double realTurnkD = 0.025;

    public static final double replayDrivekS = 0.1;
    public static final double replayDrivekV = 0.13;
    public static final double replayDrivekP = 0.05;
    public static final double replayDrivekI = 0;
    public static final double replayDrivekD = 0;
    public static final double replayTurnkP = 7.0;
    public static final double replayTurnkI = 0;
    public static final double replayTurnkD = 0;

    public static final double simDrivekS = 0.0;
    public static final double simDrivekV = 0.13;
    public static final double simDrivekP = 0.1;
    public static final double simDrivekI = 0;
    public static final double simDrivekD = 0;
    public static final double simTurnkP = 10.0;
    public static final double simTurnkI = 0;
    public static final double simTurnkD = 0;

    public static final double WHEEL_RADIUS = Units.inchesToMeters(2.0);
  }

  public final class SimModuleConstants{
    public static final double LOOP_PERIOD_SECS = 0.02;
    public static final double driveGearing = 6.75;
    public static final double turnGearing = 150.0/7.0;
    public static final double drivejKgMetersSquared = 0.025;
    public static final double turnjKgMetersSquared = 0.004;
    public static final double maxVolts = 12.0;
  }

  public final class TalonFXModuleConstants {
    public static final double driveGearRatio = (50.0/14.0) * (17.0/27.0) * (45.0/15.0);
    public static final double turnGearRatio = (150.0/7.0);
    public static final boolean turnMotorInvert = true;
    //device IDs from skywalkers-offseason
    //absolute encoder offsets
    public static final double odometryUpdateFreqHz = 100.0;
    public static final double otherUpdateFreqHz = 50.0;

    public static final double DriveStatorCurrentLimit = 40.0;
    public static final double TurnStatorCurrentLimit = 30.0;
    public static final boolean driveBrakeMode = true;
    public static final boolean turnBrakeMode = true;

    public static final boolean[] driveEncoderInverts = {true, false, true, false}; 
    public static final boolean[] turnEncoderInverts = {true, true, true, true};

    public static final class Mod0 { //frontleft
      public static final int driveMotorID = 10;
      public static final int angleMotorID = 11;
      public static final int canCoderID = 12;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(348.60);
    }

    public static final class Mod1 { //frontright
      public static final int driveMotorID = 1;
      public static final int angleMotorID = 2;
      public static final int canCoderID = 3;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(293.6);
    }

    public static final class Mod2 { //backleft
      public static final int driveMotorID = 7;
      public static final int angleMotorID = 8;
      public static final int canCoderID = 9;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(69.34);
    }

    public static final class Mod3 { //backright
      public static final int driveMotorID = 4;
      public static final int angleMotorID = 5;
      public static final int canCoderID = 6;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(320.18);
    }

  }

  public final class ElevatorSimConstants {
    public static final double gearing = 1.5;
    public static final double jKgMetersSquared = 0.004;
    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double maxVolts = 12.0;
  }

  public final class ElevatorConstants {
    public static final double gearRatio = 10.0*22.0/12.0; //from skywalkers-offseason
    public static final int kelevatorTicksPerRotation = 2048;
    public static final double kSpoolDiameter = Units.inchesToMeters(1.751);
    public static final double kDistancePerRevolution = Math.PI * kSpoolDiameter;
    public static final double kPositionConversionFactor = kDistancePerRevolution/(gearRatio * kelevatorTicksPerRotation) * 2.53715;
    public static final double kVelocityConversionFactor = kPositionConversionFactor * 10.0;

    public static final double kBottomLimit = 0.0;
    public static final double kTopLimit = 1.45;

    public static final double kVUp = 6.17;
    public static final double kSUp = 0.999;

    public static final double kVDown = 6.41;
    public static final double kSDown = -0.13;
    
    public static final double kCurrentThreshold = 10.00;
    public static final double kHomingSpeed = -0.07;

    public static final double kMountAngleRadians = 0.9599;

    public static final int kLeftElevatorPort = 31;
    public static final int kRightElevatorPort = 30;
    public static final double StatorCurrentLimit = 30.0;
    public static final double kP = 30.00;
    public static final double kMaxVel = 1.5;
    public static final double kMaxAcc = 2.0;
    public static final boolean kLeftInverted = false;
    public static final boolean kRightInverted = true;
    public static final double kMaxElevatorSpeed = 0.5; //from skywalkers-offseason but ???
  }

  public final class ArmConstants {
    public static final int kArmPort = 23;
    public static final double kP = 10.00;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final boolean kInverted = false;
    public static final double gearRatio = 45.0*44.0/18.0;
    public static final double kAbsEncoderOffset = 0;
    public static final int kArmAbsoluteEncoderPort = 16;
    public static final double kMaxArmSpeed = 0.50; // ??
    public static final int kArmTicksPerRotation = 2048; // ??
    public static final double kPositionConversionFactor = 2 * Math.PI / (kArmTicksPerRotation * gearRatio);
    public static final double kVelocityConversionFactor = kPositionConversionFactor * 10.0;
  }

  public final class IntakeConstants {
    public static final int kIntakePort = 22; // ??
    public static final double kMaxIntakeSpeed = 0.7; // ?? 0.4
    public static final double kMaxOuttakeSpeed = -0.6; // ??
    public static final double kHoldSpeed = 0.07; //THIS ONE, 0.09

    public static double kSpeedUpFailTime = 0.75; //seconds it tries to speed up
    public static double kOutFailTime = 0.75; //seconds it tries to outtake

    public static double conePieceHeldThreshold = 12; //20
    public static double cubePieceHeldThreshold = 12; //20
    public static boolean differentialIntake = true;
    public static int tofPort = 15;
    
    public static double threshold(double speed) {
      speed = Math.abs(speed);
          return (speed * 20850) - 1365;
      }
  }
}
