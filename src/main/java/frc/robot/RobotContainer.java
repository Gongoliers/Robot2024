package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.Telemetry;
import frc.robot.arm.Arm;
import frc.robot.auto.Auto;
import frc.robot.intake.Intake;
import frc.robot.odometry.Odometry;
import frc.robot.shooter.Shooter;
import frc.robot.superstructure.Superstructure;
import frc.robot.superstructure.SuperstructureMechanism;
import frc.robot.superstructure.SuperstructureState;
import frc.robot.swerve.Swerve;

/** Robot container. */
public class RobotContainer {

  /** Robot container singleton. */
  private static RobotContainer instance = null;

  /** Arm subsystem reference. */
  private final Arm arm;

  /** Auto subsystem reference. */
  private final Auto auto;

  /** Intake subsystem reference. */
  private final Intake intake;

  /** Odometry subsystem reference. */
  private final Odometry odometry;

  /** Shooter subsystem reference. */
  private final Shooter shooter;

  /** Superstructure subsystem reference. */
  private final Superstructure superstructure;

  /** Swerve subsystem reference. */
  private final Swerve swerve;

  /** Driver controller. */
  private final CommandXboxController driverController;

  /** Operator controller. */
  private final CommandXboxController operatorController;

  /** Initializes the robot container. */
  private RobotContainer() {
    arm = Arm.getInstance();
    auto = Auto.getInstance();
    intake = Intake.getInstance();
    odometry = Odometry.getInstance();
    shooter = Shooter.getInstance();
    superstructure = Superstructure.getInstance();
    swerve = Swerve.getInstance();

    driverController = new CommandXboxController(0);

    operatorController = new CommandXboxController(1);

    initializeTelemetry();
    configureDefaultCommands();
    configureBindings();
  }

  /**
   * Returns the robot container.
   *
   * @return the robot container.
   */
  public static RobotContainer getInstance() {
    if (instance == null) {
      instance = new RobotContainer();
    }

    return instance;
  }

  /** Initializes subsystem telemetry. */
  private void initializeTelemetry() {
    Telemetry.initializeTabs(arm, auto, intake, odometry, shooter, superstructure, swerve);
    SmartDashboard.putData("Superstructure", SuperstructureMechanism.getInstance().getMechanism());
  }

  /** Configures subsystem default commands. */
  private void configureDefaultCommands() {
    swerve.setDefaultCommand(swerve.teleopDrive(driverController));
  }

  /** Configures controller bindings. */
  private void configureBindings() {
    driverController.a().whileTrue(swerve.forwards());
    driverController.b().whileTrue(swerve.sideways());
    driverController.x().whileTrue(swerve.cross());

    driverController.y().onTrue(odometry.tare());

    operatorController.leftBumper().onTrue(superstructure.eject());
    operatorController.leftTrigger().onTrue(superstructure.intake());

    operatorController.rightBumper().onTrue(superstructure.shoot(SuperstructureState.SUBWOOFER));
    operatorController.rightTrigger().onTrue(superstructure.prepare(SuperstructureState.SUBWOOFER));

    operatorController.a().onTrue(superstructure.amp());
    operatorController.x().onTrue(superstructure.stow());
    operatorController.y().onTrue(superstructure.skim());
  }

  /**
   * Returns the command to run during the autonomous period.
   *
   * @return the command to run during the autonomous period.
   */
  public Command getAutonomousCommand() {
    return auto.getSelectedCommand();
  }
}
