package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

  private final Arm arm;
  private final Auto auto;
  private final Climber climber;
  private final Intake intake;
  private final Odometry odometry;
  private final Shooter shooter;
  private final Swerve swerve;

  private final CommandXboxController driverController, operatorController;

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
    operatorController = new CommandXboxController(1);

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
    Telemetry.initializeTabs(arm, auto, climber, intake, odometry, shooter, swerve);
    SmartDashboard.putData("Mechanism", RobotMechanisms.getInstance().getMechanism());
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
        .whileFalse((auto.stow()));
    operatorController
        .leftBumper()
        .whileTrue(intake.out().andThen(intake.outtake()))
        .whileFalse(auto.stow());

    operatorController
        .rightTrigger()
        .whileTrue(auto.readyShoot().andThen(shooter.spin()))
        .whileFalse(auto.stow());
    operatorController.rightBumper().whileTrue(shooter.serialize());

    operatorController.povUp().whileTrue(climber.up());
    operatorController
        .povDown()
        .whileTrue(Commands.parallel(auto.readyIntake().repeatedly(), climber.down()));

    operatorController.a().whileTrue(arm.amp()).whileFalse(auto.stow());
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
