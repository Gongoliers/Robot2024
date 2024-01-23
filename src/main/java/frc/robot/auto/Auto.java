package frc.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.Subsystem;
import frc.lib.Telemetry;
import frc.robot.odometry.Odometry;
import frc.robot.swerve.Swerve;
import frc.robot.swerve.SwerveConstants;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;

/** Subsystem class for the auto subsystem. */
public class Auto extends Subsystem {

  /** Instance variable for the auto subsystem singleton. */
  private static Auto instance = null;

  private final SendableChooser<Command> autoChooser;

  /** Creates a new instance of the auto subsystem. */
  private Auto() {
    Supplier<Pose2d> robotPositionSupplier = () -> Odometry.getInstance().getPosition();

    Consumer<Pose2d> robotPositionConsumer =
        position -> Odometry.getInstance().setPosition(position);

    Supplier<ChassisSpeeds> swerveChassisSpeedsSupplier =
        () -> Swerve.getInstance().getChassisSpeeds();

    Consumer<ChassisSpeeds> swerveChassisSpeedsConsumer =
        chassisSpeeds -> Swerve.getInstance().setChassisSpeeds(chassisSpeeds);

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
        this::shouldFlipPath,
        this);

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
   * Determines whether a path should be flipped.
   *
   * @return whether the path should be flipped.
   */
  private boolean shouldFlipPath() {
    Optional<Alliance> alliance = DriverStation.getAlliance();

    if (alliance.isEmpty()) return false;

    return alliance.get() == DriverStation.Alliance.Red;
  }

  /**
   * Gets the command to run during the autonomous period.
   *
   * @return the command to run during the autonomous period.
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
