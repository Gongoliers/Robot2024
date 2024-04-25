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
import frc.robot.RobotConstants;

/** Subsystem class for the swerve subsystem. */
public class Swerve extends Subsystem {

  /** Instance variable for the swerve subsystem singleton. */
  private static Swerve instance = null;

  /** Swerve modules controlled by the swerve subsystem. */
  private final SwerveModuleIO[] swerveModules = new SwerveModuleIO[4];

  /** Swerve kinematics. */
  private final SwerveDriveKinematics swerveKinematics;

  /** Creates a new instance of the swerve subsystem. */
  private Swerve() {
    swerveModules[0] = SwerveFactory.createModule(SwerveConstants.NORTH_WEST_MODULE_CONFIG);
    swerveModules[1] = SwerveFactory.createModule(SwerveConstants.NORTH_EAST_MODULE_CONFIG);
    swerveModules[2] = SwerveFactory.createModule(SwerveConstants.SOUTH_EAST_MODULE_CONFIG);
    swerveModules[3] = SwerveFactory.createModule(SwerveConstants.SOUTH_WEST_MODULE_CONFIG);

    swerveKinematics =
        new SwerveDriveKinematics(
            SwerveConstants.NORTH_WEST_MODULE_CONFIG.position(),
            SwerveConstants.NORTH_EAST_MODULE_CONFIG.position(),
            SwerveConstants.SOUTH_EAST_MODULE_CONFIG.position(),
            SwerveConstants.SOUTH_WEST_MODULE_CONFIG.position());
  }

  /**
   * Gets the instance of the swerve subsystem.
   *
   * @return the instance of the swerve subsystem.
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

    translationConstants.addDouble(
        "Maximum Attainable Velocity (mps)", () -> SwerveConstants.MAXIMUM_ATTAINABLE_SPEED);
    translationConstants.addDouble("Maximum Velocity (mps)", () -> SwerveConstants.MAXIMUM_SPEED);
    translationConstants.addDouble(
        "Maximum Accleration (mpsps)", () -> SwerveConstants.MAXIMUM_ACCELERATION);
    translationConstants.addDouble(
        "Wheel Circumference (m)", () -> SwerveConstants.MK4iConstants.WHEEL_CIRCUMFERENCE);

    ShuffleboardLayout rotationConstants = Telemetry.addColumn(tab, "Rotation Constants");
    rotationConstants.addDouble(
        "Maximum Attainable Velocity (rps)",
        () -> SwerveConstants.MAXIMUM_ATTAINABLE_ROTATION_SPEED.getRotations());
    rotationConstants.addDouble(
        "Maximum Velocity (rps)", () -> SwerveConstants.MAXIMUM_ROTATION_SPEED.getRotations());
    rotationConstants.addDouble(
        "Maximum Acceleration (rpsps)",
        () -> SwerveConstants.MAXIMUM_ROTATION_ACCELERATION.getRotations());

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
   * Gets the swerve's kinematics.
   *
   * @return the swerve's kinematics.
   */
  public SwerveDriveKinematics getKinematics() {
    return swerveKinematics;
  }

  /**
   * Gets the state of each of the swerve's modules.
   *
   * @return the state of each of the swerve's modules.
   */
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] moduleStates = new SwerveModuleState[4];

    for (int i = 0; i < 4; i++) {
      moduleStates[i] = swerveModules[i].getState();
    }

    return moduleStates;
  }

  /**
   * Gets the setpoint of each of the swerve's modules.
   *
   * @return the setpoint of each of the swerve's modules.
   */
  public SwerveModuleState[] getModuleSetpoints() {
    SwerveModuleState[] moduleSetpoints = new SwerveModuleState[4];

    for (int i = 0; i < 4; i++) {
      moduleSetpoints[i] = swerveModules[i].getSetpoint();
    }

    return moduleSetpoints;
  }

  /**
   * Gets the position of each of the swerve's modules.
   *
   * @return the position of each of the swerve's modules.
   */
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];

    for (int i = 0; i < 4; i++) {
      modulePositions[i] = swerveModules[i].getPosition();
    }

    return modulePositions;
  }

  /**
   * Gets the swerve's speeds.
   *
   * @return the swerve's speeds.
   */
  public ChassisSpeeds getChassisSpeeds() {
    return swerveKinematics.toChassisSpeeds(getModuleStates());
  }

  /**
   * Sets the swerve's speeds.
   *
   * @param speeds the swerve's speeds.
   */
  public void setChassisSpeeds(ChassisSpeeds speeds) {
    speeds = ChassisSpeeds.discretize(speeds, RobotConstants.PERIODIC_DURATION);

    SwerveModuleState[] setpoints = swerveKinematics.toSwerveModuleStates(speeds);

    setSetpoints(setpoints, true);
  }

  /**
   * Sets each of the swerve modules' setpoints.
   *
   * @param setpoints the setpoints for each of the swerve's modules.
   * @param lazy if true, optimize the module setpoint.
   */
  public void setSetpoints(SwerveModuleState[] setpoints, boolean lazy) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        setpoints, SwerveConstants.MAXIMUM_ATTAINABLE_SPEED);

    for (int i = 0; i < 4; i++) {
      swerveModules[i].setSetpoint(setpoints[i], lazy);
    }
  }

  /**
   * Returns a command that drives the swerve using an Xbox controller.
   *
   * @param controller the Xbox controller to use.
   * @return a command that drives the swerve using an Xbox controller.
   */
  public Command driveWithController(CommandXboxController controller) {
    return new DriveCommand(controller);
  }

  /**
   * Set the steer motor setpoints for each of the swerve modules.
   *
   * @param orientations orientations for each swerve modules.
   * @return a command that orients all swerve modules.
   */
  public Command orientModules(Rotation2d[] orientations) {
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

  public Command forwards() {
    return orientModules(
        new Rotation2d[] {
          Rotation2d.fromDegrees(0.0),
          Rotation2d.fromDegrees(0.0),
          Rotation2d.fromDegrees(0.0),
          Rotation2d.fromDegrees(0.0)
        });
  }

  public Command sideways() {
    return orientModules(
        new Rotation2d[] {
          Rotation2d.fromDegrees(90.0),
          Rotation2d.fromDegrees(90.0),
          Rotation2d.fromDegrees(90.0),
          Rotation2d.fromDegrees(90.0)
        });
  }

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
