package frc.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.AllianceFlipHelper;
import frc.lib.Subsystem;
import frc.lib.Telemetry;
import frc.robot.odometry.Odometry;
import frc.robot.superstructure.Superstructure;
import frc.robot.swerve.Swerve;
import frc.robot.swerve.SwerveConstants;
import frc.robot.swerve.SwerveFactory;

/** Auto subsystem. */
public class Auto extends Subsystem {

  /** Auto singleton. */
  private static Auto instance = null;

  /** Odometry subsystem reference. */
  private final Odometry odometry;

  /** Superstructure subsystem reference. */
  private final Superstructure superstructure;

  /** Swerve subsystem reference. */
  private final Swerve swerve;

  /** Auto command chooser. */
  private final SendableChooser<Command> autoChooser;

  /** Initializes the auto subsystem. */
  private Auto() {
    odometry = Odometry.getInstance();
    superstructure = Superstructure.getInstance();
    swerve = Swerve.getInstance();

    AutoBuilder.configureHolonomic(
        odometry::getPosition,
        odometry::setPosition,
        swerve::getChassisSpeeds,
        swerve::setChassisSpeeds,
        new HolonomicPathFollowerConfig(
            new PIDConstants(1.0),
            new PIDConstants(1.0),
            SwerveConstants.MAXIMUM_SPEED,
            SwerveFactory.createNorthEastModuleTranslation().getNorm(), // TODO
            new ReplanningConfig()),
        AllianceFlipHelper::shouldFlip,
        swerve);

    NamedCommands.registerCommand("stow", superstructure.stow());
    NamedCommands.registerCommand(
        "shoot", superstructure.subwoofer().withTimeout(1.5)); // 1 second could work
    NamedCommands.registerCommand("intake", superstructure.intakeInstant());

    autoChooser = AutoBuilder.buildAutoChooser();
  }

  /**
   * Returns the auto subsystem instance.
   *
   * @return the auto subsystem instance.
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
   * Returns the selected auto command.
   *
   * @return the selected auto command.
   */
  public Command getSelectedCommand() {
    return autoChooser.getSelected();
  }
}
