package frc.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.AllianceFlipHelper;
import frc.lib.Subsystem;
import frc.lib.Telemetry;
import frc.robot.arm.Arm;
import frc.robot.arm.ArmState;
import frc.robot.intake.Intake;
import frc.robot.odometry.Odometry;
import frc.robot.shooter.Shooter;
import frc.robot.swerve.Swerve;
import frc.robot.swerve.SwerveConstants;
import java.util.function.Consumer;
import java.util.function.Supplier;

/** Subsystem class for the auto subsystem. */
public class Auto extends Subsystem {

  /** Instance variable for the auto subsystem singleton. */
  private static Auto instance = null;

  /** Reference to the arm subsystem. */
  private final Arm arm;

  /** Reference to the intake subsystem. */
  private final Intake intake;

  /** Reference to the odometry subsystem. */
  private final Odometry odometry;

  /** Reference to the shooter subssytem. */
  private final Shooter shooter;

  /** Reference to the swerve subsystem. */
  private final Swerve swerve;

  /** Sendable chooser for the subsystem. */
  private final SendableChooser<Command> autoChooser;

  /** Creates a new instance of the auto subsystem. */
  private Auto() {
    arm = Arm.getInstance();
    intake = Intake.getInstance();
    odometry = Odometry.getInstance();
    shooter = Shooter.getInstance();
    swerve = Swerve.getInstance();

    Supplier<Pose2d> robotPositionSupplier = () -> odometry.getPosition();

    Consumer<Pose2d> robotPositionConsumer = position -> odometry.setPosition(position);

    Supplier<ChassisSpeeds> swerveChassisSpeedsSupplier = () -> swerve.getChassisSpeeds();

    Consumer<ChassisSpeeds> swerveChassisSpeedsConsumer =
        chassisSpeeds -> swerve.setChassisSpeeds(chassisSpeeds);

    HolonomicPathFollowerConfig holonomicPathFollowerConfig =
        new HolonomicPathFollowerConfig(
            new PIDConstants(1.0),
            new PIDConstants(1.0),
            SwerveConstants.MAXIMUM_ATTAINABLE_SPEED,
            SwerveConstants.NORTH_EAST_MODULE_CONFIG.position().getNorm(),
            new ReplanningConfig());

    AutoBuilder.configureHolonomic(
        robotPositionSupplier,
        robotPositionConsumer,
        swerveChassisSpeedsSupplier,
        swerveChassisSpeedsConsumer,
        holonomicPathFollowerConfig,
        AllianceFlipHelper::shouldFlip,
        swerve);

    NamedCommands.registerCommand("home", Arm.getInstance().home());
    NamedCommands.registerCommand("stow", stow());
    NamedCommands.registerCommand("readyIntake", readyIntake());
    NamedCommands.registerCommand("intakeNote", intakeNote());
    NamedCommands.registerCommand("readyShoot", readyShoot());
    NamedCommands.registerCommand("shootNote", shootNote());

    autoChooser = AutoBuilder.buildAutoChooser();
  }

  /**
   * Gets the instance of the auto subsystem.
   *
   * @return the instance of the auto subsystem.
   */
  public static Auto getInstance() {
    if (instance == null) {
      instance = new Auto();
    }

    return instance;
  }

  @Override
  public void periodic() {}

  @Override
  public void addToShuffleboard(ShuffleboardTab tab) {
    Telemetry.makeFullscreen(tab.add("Auto Chooser", autoChooser));
  }

  /**
   * Gets the command to run during the autonomous period.
   *
   * @return the command to run during the autonomous period.
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  /**
   * Gets the chooser for the command to run during the autonomous period.
   *
   * @return the chooser for the command to run during the autonomous period.
   */
  public SendableChooser<Command> getAutonomousChooser() {
    return autoChooser;
  }

  public Command readyIntake() {
    double seconds = 3.0;

    return Commands.parallel(
            Commands.waitUntil(intake::isNotStowed)
                .andThen(arm.moveShoulderThenWrist(ArmState.INTAKE)),
            intake.out())
        .withTimeout(seconds);
  }

  public Command intakeNote() {
    return readyIntake().andThen(Commands.parallel(intake.intake(), shooter.intake()));
  }

  public Command stow() {
    double seconds = 2.0;

    return Commands.parallel(
            arm.moveWristThenShoulder(ArmState.STOW),
            Commands.waitUntil(() -> arm.getPosition().at(ArmState.STOW))
                .withTimeout(2.0)
                .andThen(intake.in()))
        .withTimeout(seconds);
  }

  public Command readyShoot() {
    double seconds = 3.0;

    return Commands.parallel(
            Commands.waitUntil(intake::isNotStowed).andThen(arm.moveWrist(ArmState.SHOOT)),
            intake.out())
        .withTimeout(seconds);
  }

  public Command shootNote() {
    return readyShoot().andThen(shooter.autoShoot());
  }
}
