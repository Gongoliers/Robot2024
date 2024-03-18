package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.Telemetry;
import frc.robot.arm.Arm;
import frc.robot.auto.Auto;
import frc.robot.climber.Climber;
import frc.robot.intake.Intake;
import frc.robot.odometry.Odometry;
import frc.robot.shooter.Shooter;
import frc.robot.swerve.Swerve;

/** Initializes subsystems and commands. */
public class RobotContainer {

  /** Instance variable for the robot container singleton. */
  public static RobotContainer instance = null;

  private final Arm arm;
  private final Auto auto;
  private final Climber climber;
  private final Intake intake;
  private final Odometry odometry;
  private final Shooter shooter;
  private final Swerve swerve;

  private final CommandXboxController driverController;

  /** Creates a new instance of the robot container. */
  private RobotContainer() {
    arm = Arm.getInstance();
    auto = Auto.getInstance();
    climber = Climber.getInstance();
    intake = Intake.getInstance();
    odometry = Odometry.getInstance();
    shooter = Shooter.getInstance();
    swerve = Swerve.getInstance();

    driverController = new CommandXboxController(0);

    initializeTelemetry();
    configureDefaultCommands();
    configureBindings();
  }

  /**
   * Gets the instance of the robot container.
   *
   * @return the instance of the robot container.
   */
  public static RobotContainer getInstance() {
    if (instance == null) {
      instance = new RobotContainer();
    }

    return instance;
  }

  /** Initializes subsystem telemetry. */
  private void initializeTelemetry() {
    Telemetry.initializeTabs(arm, auto, climber, intake, odometry, shooter, swerve);
    SmartDashboard.putData("Mechanism", RobotMechanisms.getInstance().getMechanism());
  }

  /** Configures subsystem default commands. */
  private void configureDefaultCommands() {
    swerve.setDefaultCommand(swerve.driveWithController(driverController));
  }

  /** Configures controller bindings. */
  private void configureBindings() {
    driverController.y().onTrue(odometry.tare());
  }

  /**
   * Gets the command to run during the autonomous period.
   *
   * @return the command to run during the autonomous period.
   */
  public Command getAutonomousCommand() {
    return auto.getAutonomousCommand();
  }
}
