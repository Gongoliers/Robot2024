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
import frc.robot.swerve.DriveCommand;
import frc.robot.swerve.Swerve;

/** Initializes subsystems and commands. */
public class RobotContainer {

  /** Instance variable for the robot container singleton. */
  public static RobotContainer instance = null;

  private final Arm arm = Arm.getInstance();
  private final Auto auto = Auto.getInstance();
  private final Climber climber = Climber.getInstance();
  private final Intake intake = Intake.getInstance();
  private final Odometry odometry = Odometry.getInstance();
  private final Shooter shooter = Shooter.getInstance();
  private final Swerve swerve = Swerve.getInstance();

  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  /** Creates a new instance of the robot container. */
  private RobotContainer() {
    initializeTelemetry();
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
    if (RobotConstants.USE_TELEMETRY) {
      Telemetry.initializeShuffleboards(arm, auto, climber, intake, odometry, shooter, swerve);
      SmartDashboard.putData("Arm Mechanism", RobotMechanisms.getInstance().getMechanism());
    }

    SmartDashboard.putData(auto.getAutonomousChooser());
  }

  /** Configures operator controller bindings. */
  private void configureBindings() {
    swerve.setDefaultCommand(new DriveCommand(driverController));

    driverController.a().whileTrue(swerve.forwards());
    driverController.b().whileTrue(swerve.sideways());
    driverController.x().whileTrue(swerve.cross());
    driverController.y().onTrue(odometry.tare());

    operatorController
        .leftTrigger()
        .whileTrue(auto.readyIntake().andThen(auto.intakeNote()))
        .onFalse(auto.stow());

    operatorController.rightTrigger().whileTrue(auto.shootNote()).onFalse(auto.stow());

    operatorController.a().whileTrue(shooter.shoot());

    // TODO When stowing, the wrist encoder drifts by ~30% of the shoulder velocity
    // TODO This **will** throw off wrist angles for the rest of the match
    // operatorController.a().whileTrue(arm.amp()).onFalse(auto.stow());
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
