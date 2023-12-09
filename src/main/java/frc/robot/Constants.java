package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import frc.robot.lightstrip.LedState;
import frc.robot.lightstrip.TempLedState;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Mode currentMode = Mode.REAL;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final double kDeadband = 0.1;

  public final class DriveConstants {
    public static final double DEADBAND = 0.15;
    public static final double MAX_LINEAR_SPEED = 5.00; //meters per second
    public static final double TRACK_WIDTH_X = Units.inchesToMeters(24.0);
    public static final double TRACK_WIDTH_Y = Units.inchesToMeters(24.0);
    public static final double DRIVE_BASE_RADIUS = Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
    public static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;

    public static final int PigeonID = 13;
    public static final double kSlowmode = 0.6;
  }

  public static final class OIConstants {
    public static final double kDeadband = 0.15;

    public static final int kDriverControllerPort = 0;
    public static final int kDriverControllerPort2 = 1;

    public static final int kDriverYAxis = 1;
    public static final int kDriverXAxis = 0;
    public static final int kDriverRotAxis = 4;
    public static final int kDriverFieldOrientedButtonIdx = 1;
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
    public static final double gearing = 10.0*22.0/12.0;
    public static final double jKgMetersSquared = 0.25;
    public static final double kP = 30.0;
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
    public static final double kP = 30.0; //was 30
    public static final double kMaxVel = 1.5;
    public static final double kMaxAcc = 2.0;
    public static final boolean kLeftInverted = false;
    public static final boolean kRightInverted = true;
    public static final double kMaxElevatorSpeed = 0.5; //from skywalkers-offseason but ???

    public static final double tiltAngle = Math.PI/2; //wrong
    public static final double carraigeMassKg = 1.5; //wrong
    public static final double drumRadiusMeters = 0.02;
    public static final double minHeight = 0;
    public static final double maxHeight = 4.5;

    public static final double kMaxVolts = 8.0;
    public static final double kMaxVoltsSim = 12.0;
  }

  public final class ArmConstants {
    public static final int kArmPort = 23;
    public static final double kP = 10.00;
    public static final double kPsim = 6.00;
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
    public static final double armLengthMeters = 0.25; //idk
    public static final double minAngleRads = -0.3; //need to redo these
    public static final double maxAngleRads = 1.85;
    public static final double kMaxVoltsSim = 6.00; //from offseason
    public static final double kMaxVolts = 6.00;
    public static final double jKgMetersSquared = 0.05; //???
    public static final double StatorCurrentLimit = 15.00;

    public static final double kVUp = 1.95;
    public static final double kSUp = 0.613;
    public static final double kVDown = 1.92;
    public static final double kSDown = -0.424;

    public static final double kMaxVel = 3; //from offseason
    public static final double kMaxAcc = 4;
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

  public static final class LimelightConstants{
    public static double kPx = 5; //meters
    public static double kPy = 5; //meters
    public static double kPr = 0.2; //degrees
    public static double kDx = 0;
    public static double kDy = 0;
    public static double kDr = 0;
    public static double kIx = 0;
    public static double kIy = 0;
    public static double kIr = 0;

    public static double xclamp = 0.8; //maximum clamped speed is 0.6 + 0.5 (min)
    public static double yclamp = 0.8;
    public static double rclamp = 2;

    //public static double tagheight = 0.49; //19.3 inches to meters

    public static double xtolerance = 0.02; 
    public static double ytolerance = 0.02; 
    public static double rtolerance = 2; //degrees

    public static double mountingangle = 0; //for adjustable camera
    public static double cameraheight = Units.inchesToMeters(17.5); //14-ish inches, to meters, REDO
    public static double RTheight = Units.inchesToMeters(22.55); //game manual 24.125
    public static double objectHeight = Units.inchesToMeters(0);

    public static double limelightOffsetCenter = Units.inchesToMeters(10.5);

    public static final class SecondStageConeConstants{
        public static double targetXMeters = 0.92;
        public static double targetYMeters = 0;
        public static double targetRDeg = 0;
    }
}

public static final class DashboardConstants {
    public static boolean SwerveDebugging = true;
    public static boolean SwerveDriver = true;
    public static boolean ArmDebugging = true;
    public static boolean ArmDriver = true;
    public static boolean ElevatorDebugging = true;
    public static boolean ElevatorDriver = true;
    public static boolean IntakeDebugging = true;
    public static boolean IntakeDriver = true;
    public static boolean AutoDebugging = true;
    public static boolean AutoDriver = true;
    public static boolean TeleDebugging = true;
    public static boolean TeleDriver = true;
    public static boolean LimelightDebugging = true;
    public static boolean LimelightDriver = true;
}

public static final class lightstripConstants {
    public static int redPort = 1;
    public static int greenPort = 2;
    public static int bluePort = 0;

    public static LedState defaultState = new LedState(255, 0, 0, "Solid");
    public static TempLedState successSignal = new TempLedState(0, 255, 0, "Solid", 2);
    public static LedState coneIntake = new LedState(255, 200, 0, "Solid");
    public static LedState cubeIntake = new LedState(195, 0, 255, "Solid");
}
public static final class Presets {
  public static final Preset STOW_PRESET = new Preset(1.6, 0); // done
  public static final Preset GROUND_INTAKE_PRESET = new Preset(-0.19, 0.10); // don't use
  public static final Preset GROUND_INTAKE_CONE_PRESET = new Preset(0.22, 0);     
  public static final Preset GROUND_INTAKE_CUBE_PRESET = new Preset(0.1, 0.01); // done
  public static final Preset SUBSTATION_INTAKE_CONE_PRESET = new Preset(0.06, 1.25);
  public static final Preset SUBSTATION_INTAKE_PRESET = new Preset(0.0, 1.13);
  public static final Preset SUBSTATION_INTAKE_CUBE_PRESET = new Preset(0.45, 1.05);
  public static final Preset CONE_2ND_STAGE_PRESET = new Preset(0.54, 0.63);
  public static final Preset CONE_3RD_STAGE_PRESET = new Preset(0.48, 1.16);
  public static final Preset CUBE_2ND_STAGE_PRESET = new Preset(0.81, 0.60);
  public static final Preset CUBE_3RD_STAGE_PRESET = new Preset(0.57, 1.22);   
  public static final Preset SINGLE_SUBSTATION_CUBE = new Preset(1.85, 0);
  public static final Preset SINGLE_SUBSTATION_CONE = new Preset(0.72, 0.68);
}

}
