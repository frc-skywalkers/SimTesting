package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.JoystickSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.Macros;
import frc.robot.subsystems.Visualizer;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.arm.ArmIOTalonFX;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOTalonFX;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Intake intake;
  private final Elevator elevator;
  private final Arm arm;
  private final Visualizer visualizer;
  // private MechanismLigament2d m_elevator;
  // private MechanismLigament2d m_arm;
  // private MechanismLigament2d m_joint;

  // MechanismRoot2d root;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController controller2 = new CommandXboxController(1);

  private final JoystickSim joysticksim = new JoystickSim(0);
  private final Macros macros;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Choices");

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(0),
                new ModuleIOTalonFX(1),
                new ModuleIOTalonFX(2),
                new ModuleIOTalonFX(3));
        intake = new Intake(new IntakeIOTalonFX());
        elevator = new Elevator(new ElevatorIOTalonFX());
        arm = new Arm(new ArmIOTalonFX());
        macros = new Macros(drive, arm, elevator, intake);
        visualizer = new Visualizer(arm, elevator);
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        intake = new Intake(new IntakeIOSim());
        elevator = new Elevator(new ElevatorIOSim());
        arm = new Arm(new ArmIOSim());
        macros = new Macros(drive, arm, elevator, intake);
        visualizer = new Visualizer(arm, elevator);
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        intake = new Intake(new IntakeIOSim()); //braces thing doesnt work
        elevator = new Elevator(new ElevatorIOSim());
        arm = new Arm(new ArmIOSim());
        macros = new Macros(drive, arm, elevator, intake);
        visualizer = new Visualizer(arm, elevator);
        break;
    }

    // Mechanism2d mech = new Mechanism2d(3, 3);
    // root = mech.getRoot("angled elev", 1, 0.5);

    // m_elevator = root.append(new MechanismLigament2d("elevator", 0.15, 55));
    // m_joint =
    //     m_elevator.append(
    //         new MechanismLigament2d("joint", 0.3, -55, 1, new Color8Bit(Color.kPurple)));
    // m_arm =
    //   m_joint.append(
    //       new MechanismLigament2d("arm", 0.15, 0, 3, new Color8Bit(Color.kPink)));
  
    // SmartDashboard.putData("Mech2d", mech);
    // Logger.recordOutput("My Mechanism", mech);

    /*
    // Set up named commands for PathPlanner
    NamedCommands.registerCommand(
        "Run Flywheel",
        Commands.startEnd(
            () -> flywheel.runVelocity(flywheelSpeedInput.get()), flywheel::stop, flywheel));
    */

    // Set up auto routines
    autoChooser.addDefaultOption("Do Nothing", Commands.none());
    autoChooser.addOption("Example Auto", new PathPlannerAuto("Example Auto"));
    autoChooser.addOption("test auto", new PathPlannerAuto("Test Auto"));
    autoChooser.addOption("move elevator", elevator.goToPosition(1.1));
    autoChooser.addOption("move arm", arm.goToPosition(1.5));
    autoChooser.addOption("move to preset", macros.moveToPreset(1.1, 1.5)); 
    //arm start position is -0.3 radians rn, elevator is 0

    // Set up FF characterization routines
    autoChooser.addOption(
        "Drive FF Characterization",
        new FeedForwardCharacterization(
            drive, drive::runCharacterizationVolts, drive::getCharacterizationVelocity));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* 
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));
        controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
        controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    */

    elevator.setDefaultCommand(Commands.run(() -> {
      double linearMagnitude = MathUtil.applyDeadband(-controller.getLeftX(), Constants.kDeadband);
      linearMagnitude = Math.abs(linearMagnitude) * linearMagnitude;
      Logger.recordOutput("elev magnitude", linearMagnitude);
      elevator.runVelocity(linearMagnitude);
    }, elevator));

    arm.setDefaultCommand(Commands.run(() -> {
      double linearMagnitude = MathUtil.applyDeadband(-controller.getLeftY(), Constants.kDeadband);
      linearMagnitude = Math.abs(linearMagnitude) * linearMagnitude;
      arm.runVelocity(linearMagnitude);
    }, arm));

    visualizer.setDefaultCommand(Commands.run(() -> {
      visualizer.setElevatorLength(elevator.getPosition());
      visualizer.setArmAngle(arm.getPosition());
      Logger.recordOutput("observed elev length", elevator.getPosition());
      Logger.recordOutput("observed arm angle", arm.getPosition());
      Logger.recordOutput("My Mechanism", visualizer.getMech());
    }, visualizer));
    
    controller.y().onTrue(Commands.runOnce(() -> intake.moveIn()));
    controller.x().onTrue(Commands.runOnce(() -> intake.moveOut()));
    controller.a().onTrue(Commands.runOnce(() -> intake.stop()));
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return macros.moveToPreset(1.1, 1.5);
  }
}
