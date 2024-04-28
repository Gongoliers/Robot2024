package frc.robot.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.Subsystem;
import frc.lib.Telemetry;
import frc.lib.controller.SwerveModuleIO;
import frc.robot.RobotConstants;

/** Swerve subsystem. */
public class Swerve extends Subsystem {

  /** Swerve subsystem singleton. */
  private static Swerve instance = null;

  /** Swerve modules. */
  private final SwerveModuleIO[] swerveModules = new SwerveModuleIO[4];

  /** Swerve kinematics. */
  private final SwerveDriveKinematics swerveKinematics;

  /** Initializes the swerve subsystem and configures swerve hardware. */
  private Swerve() {
    swerveModules[0] = SwerveFactory.createNorthWestModule();
    swerveModules[1] = SwerveFactory.createNorthEastModule();
    swerveModules[2] = SwerveFactory.createSouthEastModule();
    swerveModules[3] = SwerveFactory.createSouthWestModule();

    swerveKinematics =
        new SwerveDriveKinematics(
            SwerveFactory.createNorthWestModuleTranslation(),
            SwerveFactory.createNorthEastModuleTranslation(),
            SwerveFactory.createSouthEastModuleTranslation(),
            SwerveFactory.createSouthWestModuleTranslation());
  }

  /**
   * Returns the swerve subsystem instance.
   *
   * @return the swerve subsystem instance.
   */
  public static Swerve getInstance() {
    if (instance == null) {
      instance = new Swerve();
    }

    return instance;
  }

  @Override
  public void periodic() {}

  @Override
  public void addToShuffleboard(ShuffleboardTab tab) {
    ShuffleboardLayout translationConstants = Telemetry.addColumn(tab, "Translation Constants");

    translationConstants.addDouble("Maximum Velocity (mps)", () -> SwerveConstants.MAXIMUM_SPEED);
    translationConstants.addDouble(
        "Maximum Accleration (mpsps)", () -> SwerveConstants.MAXIMUM_ACCELERATION);
    translationConstants.addDouble(
        "Wheel Circumference (m)", () -> SwerveConstants.MK4iConstants.WHEEL_CIRCUMFERENCE);

    ShuffleboardLayout rotationConstants = Telemetry.addColumn(tab, "Rotation Constants");
    rotationConstants.addDouble(
        "Maximum Velocity (rps)", () -> SwerveConstants.MAXIMUM_ROTATION_SPEED.getRotations());

    Telemetry.addSwerveModuleStates(tab, "Swerve Module States", this::getModuleStates);
    Telemetry.addSwerveModuleStates(tab, "Swerve Module Setpoints", this::getModuleSetpoints);

    for (int i = 0; i < 4; i++) {
      SwerveModuleIO swerveModule = swerveModules[i];

      ShuffleboardLayout swerveModuleColumn = Telemetry.addColumn(tab, "Module " + i);

      swerveModuleColumn.addDouble("Angle (deg)", () -> swerveModule.getState().angle.getDegrees());
      swerveModuleColumn.addDouble(
          "Velocity (mps)", () -> swerveModule.getState().speedMetersPerSecond);
      swerveModuleColumn.addDouble(
          "Setpoint Angle (deg)", () -> swerveModule.getSetpoint().angle.getDegrees());
      swerveModuleColumn.addDouble(
          "Setpoint Velocity (mps)", () -> swerveModule.getSetpoint().speedMetersPerSecond);
    }
  }

  /**
   * Returns the swerve kinematics.
   *
   * @return the swerve kinematics.
   */
  public SwerveDriveKinematics getKinematics() {
    return swerveKinematics;
  }

  /**
   * Returns the module states.
   *
   * @return the module states.
   */
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] moduleStates = new SwerveModuleState[4];

    for (int i = 0; i < 4; i++) {
      moduleStates[i] = swerveModules[i].getState();
    }

    return moduleStates;
  }

  /**
   * Returns the module setpoints.
   *
   * @return the module setpoints.
   */
  public SwerveModuleState[] getModuleSetpoints() {
    SwerveModuleState[] moduleSetpoints = new SwerveModuleState[4];

    for (int i = 0; i < 4; i++) {
      moduleSetpoints[i] = swerveModules[i].getSetpoint();
    }

    return moduleSetpoints;
  }

  /**
   * Returns the module positions.
   *
   * @return the module positions.
   */
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];

    for (int i = 0; i < 4; i++) {
      modulePositions[i] = swerveModules[i].getPosition();
    }

    return modulePositions;
  }

  /**
   * Returns the chassis speeds.
   *
   * @return the chassis speeds.
   */
  public ChassisSpeeds getChassisSpeeds() {
    return swerveKinematics.toChassisSpeeds(getModuleStates());
  }

  /**
   * Sets the swerve speeds.
   *
   * @param speeds the swerve speeds.
   */
  public void setChassisSpeeds(ChassisSpeeds speeds) {
    speeds = ChassisSpeeds.discretize(speeds, RobotConstants.PERIODIC_DURATION);

    SwerveModuleState[] setpoints = swerveKinematics.toSwerveModuleStates(speeds);

    setSetpoints(setpoints, true);
  }

  /**
   * Sets the swerve module setpoints.
   *
   * @param setpoints the setpoints.
   * @param lazy if true, optimize the module setpoints.
   */
  public void setSetpoints(SwerveModuleState[] setpoints, boolean lazy) {
    SwerveDriveKinematics.desaturateWheelSpeeds(setpoints, SwerveConstants.MAXIMUM_SPEED);

    for (int i = 0; i < 4; i++) {
      swerveModules[i].setSetpoint(setpoints[i], lazy);
    }
  }

  /**
   * Drives the swerve using an Xbox controller.
   *
   * @param controller the Xbox controller to use.
   * @return a command that drives the swerve using an Xbox controller.
   */
  public Command driveWithController(CommandXboxController controller) {
    return new DriveCommand(controller);
  }

  /**
   * Orients the swerve modules.
   *
   * @param orientations swerve module orientations.
   * @return a command that orients the swerve modules.
   */
  private Command orientModules(Rotation2d[] orientations) {
    return run(
        () -> {
          setSetpoints(
              new SwerveModuleState[] {
                new SwerveModuleState(0.0, orientations[0]),
                new SwerveModuleState(0.0, orientations[1]),
                new SwerveModuleState(0.0, orientations[2]),
                new SwerveModuleState(0.0, orientations[3]),
              },
              false);
        });
  }

  /**
   * Orients the swerve modules forwards (+X).
   *
   * @return a command that orients the swerve modules forwards (+X).
   */
  public Command forwards() {
    return orientModules(
        new Rotation2d[] {
          Rotation2d.fromDegrees(0.0),
          Rotation2d.fromDegrees(0.0),
          Rotation2d.fromDegrees(0.0),
          Rotation2d.fromDegrees(0.0)
        });
  }

  /**
   * Orients the swerve modules sideways (+Y).
   *
   * @return a command that orients the swerve modules sideways (+Y).
   */
  public Command sideways() {
    return orientModules(
        new Rotation2d[] {
          Rotation2d.fromDegrees(90.0),
          Rotation2d.fromDegrees(90.0),
          Rotation2d.fromDegrees(90.0),
          Rotation2d.fromDegrees(90.0)
        });
  }

  /**
   * Orients the swerve modules in a cross.
   *
   * @return a command that orients the swerve modules in a cross.
   */
  public Command cross() {
    return orientModules(
        new Rotation2d[] {
          Rotation2d.fromDegrees(45.0),
          Rotation2d.fromDegrees(-45.0),
          Rotation2d.fromDegrees(45.0),
          Rotation2d.fromDegrees(-45.0)
        });
  }
}
