package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.Telemetry;
import frc.robot.arm.Arm;
import frc.robot.auto.Auto;
import frc.robot.intake.Intake;
import frc.robot.odometry.Odometry;
import frc.robot.shooter.Shooter;
import frc.robot.superstructure.Superstructure;
import frc.robot.superstructure.SuperstructureMechanism;
import frc.robot.swerve.Swerve;

/** Initializes subsystems and commands. */
public class RobotContainer {

  /** Instance variable for the robot container singleton. */
  public static RobotContainer instance = null;

  private final Arm arm;
  private final Auto auto;
  private final Intake intake;
  private final Odometry odometry;
  private final Shooter shooter;
  private final Superstructure superstructure;
  private final Swerve swerve;

  private final CommandXboxController driverController, operatorController;

  private final XboxController rumbleController; 

  /** Creates a new instance of the robot container. */
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
    rumbleController = new XboxController(1);

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
    Telemetry.initializeTabs(arm, auto, intake, odometry, shooter, superstructure, swerve);
    SmartDashboard.putData("Superstructure", SuperstructureMechanism.getInstance().getMechanism());
  }

  /** Configures subsystem default commands. */
  private void configureDefaultCommands() {
    swerve.setDefaultCommand(swerve.driveWithController(driverController));
  }

  /** Configures controller bindings. */
  private void configureBindings() {
    driverController.a().whileTrue(swerve.forwards());
    driverController.b().whileTrue(swerve.sideways());
    driverController.x().whileTrue(swerve.cross());

    driverController.y().onTrue(odometry.tare());

    //operatorController.leftBumper().onTrue(superstructure.eject());
    operatorController.leftTrigger().onTrue(superstructure.intake());

    operatorController.rightBumper().onTrue(superstructure.pass());
    operatorController.rightTrigger().onTrue(superstructure.speaker());

    operatorController.a().onTrue(superstructure.amp());
    operatorController.x().onTrue(superstructure.stow());
    operatorController.y().onTrue(superstructure.climb());

    new Trigger(intake::rollerNote).onTrue(rumbleOn()).onFalse(rumbleOff());
  }

  public void rumble(double value) {
    rumbleController.setRumble(RumbleType.kBothRumble, value);
  }

  public Command rumbleOn() {
    return Commands.runOnce(() -> rumble(1));
  }

  public Command rumbleOff() {
    return Commands.runOnce(() -> rumble(0));
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
