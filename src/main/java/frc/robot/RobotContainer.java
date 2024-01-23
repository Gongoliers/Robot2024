package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.CustomXboxController;
import frc.lib.Telemetry;
import frc.robot.arm.Arm;
import frc.robot.auto.Auto;
import frc.robot.climber.Climber;
import frc.robot.intake.Intake;
import frc.robot.lights.Lights;
import frc.robot.odometry.Odometry;
import frc.robot.shooter.Shooter;
import frc.robot.swerve.Drive;
import frc.robot.swerve.Swerve;
import frc.robot.vision.Vision;

public class RobotContainer {

  /** Instance variable for the robot container singleton. */
  public static RobotContainer instance = null;

  private final Arm arm = Arm.getInstance();
  private final Auto auto = Auto.getInstance();
  private final Climber climber = Climber.getInstance();
  private final Intake intake = Intake.getInstance();
  private final Lights lights = Lights.getInstance();
  private final Odometry odometry = Odometry.getInstance();
  private final Shooter shooter = Shooter.getInstance();
  private final Swerve swerve = Swerve.getInstance();
  private final Vision vision = Vision.getInstance();

  private final CustomXboxController driverController = new CustomXboxController(0);
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
    Telemetry.initializeShuffleboards(
        arm, auto, climber, intake, lights, odometry, shooter, swerve, vision);
  }

  /** Configures operator controller bindings. */
  private void configureBindings() {
    swerve.setDefaultCommand(new Drive(driverController));

    driverController.a().whileTrue(swerve.forwards());
    driverController.b().whileTrue(swerve.sideways());
    driverController
        .y()
        .onTrue(Commands.runOnce(() -> odometry.setRotation(Rotation2d.fromDegrees(0))));
    driverController.x().whileTrue(swerve.cross());

    operatorController.leftBumper().whileTrue(intake.intake()).whileTrue(shooter.intake());
    operatorController.leftTrigger().whileTrue(shooter.smartIntake());

    operatorController.rightBumper().whileTrue(shooter.shoot());
    operatorController.rightTrigger().whileTrue(shooter.smartShoot());
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
