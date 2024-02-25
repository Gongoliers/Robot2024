package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.Telemetry;
import frc.robot.arm.Arm;
import frc.robot.arm.ArmState;
import frc.robot.auto.Auto;
import frc.robot.climber.Climber;
import frc.robot.intake.Intake;
import frc.robot.intake.IntakeConstants.PivotMotorConstants;
import frc.robot.lights.Lights;
import frc.robot.odometry.Odometry;
import frc.robot.shooter.Shooter;
import frc.robot.swerve.DriveCommand;
import frc.robot.swerve.Swerve;
import frc.robot.vision.Vision;

/** Initializes subsystems and commands. */
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
    Telemetry.initializeShuffleboards(
        arm, auto, climber, intake, lights, odometry, shooter, swerve, vision);

    SmartDashboard.putData("Mechanism", RobotMechanisms.getInstance().getMechanism());
  }

  /** Configures operator controller bindings. */
  private void configureBindings() {
    swerve.setDefaultCommand(new DriveCommand(driverController));

    driverController.y().onTrue(odometry.tare());
    driverController.x().whileTrue(swerve.cross());

    Command toIntake = Commands.parallel(
        Commands.waitUntil(intake::isNotStowed).andThen(arm.to(ArmState.INTAKE)),
        intake.out());

    Command toStow = Commands.runOnce(() -> {
      arm.setGoal(ArmState.STOW);
      intake.setPivotGoal(PivotMotorConstants.MAXIMUM_ANGLE);
    });

    operatorController
        .leftTrigger()
        .whileTrue(
            toIntake 
                .andThen(Commands.parallel(intake.intake(), shooter.intake()))).onFalse(toStow);

    operatorController
        .rightTrigger()
        .whileTrue(arm.to(ArmState.SHOOT).andThen(shooter.shoot())).onFalse(Commands.runOnce(() -> arm.setGoal(ArmState.STOW)));
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
