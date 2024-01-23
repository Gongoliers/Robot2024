package frc.robot.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.lib.Subsystem;
import frc.lib.Telemetry;

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
    Telemetry.addSwerveModuleStates(tab, "Swerve Module States", this::getModuleStates);
    Telemetry.addSwerveModuleStates(tab, "Swerve Module Setpoints", this::getModuleSetpoints);

    for (int i = 0; i < 4; i++) {
      SwerveModuleIO swerveModule = swerveModules[i];

      ShuffleboardLayout swerveModuleColumn = Telemetry.addColumn(tab, "Module " + i);

      swerveModuleColumn.addDouble("Azimuth (deg)", () -> swerveModule.getAzimuth().getDegrees());
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
    SwerveModuleState[] setpoints = swerveKinematics.toSwerveModuleStates(speeds);

    setSetpoints(setpoints, true);
  }

  /**
   * Set the setpoints for each of the swerve's modules.
   *
   * @param setpoints the setpoints for each of the swerve's modules.
   * @param lazy if true, optimize the module setpoint.
   */
  public void setSetpoints(SwerveModuleState[] setpoints, boolean lazy) {
    SwerveDriveKinematics.desaturateWheelSpeeds(setpoints, SwerveConstants.MAXIMUM_SPEED);

    for (int i = 0; i < 4; i++) {
      swerveModules[i].setSetpoint(setpoints[i], lazy);
    }
  }

  /**
   * Set the steer motor setpoints for each of the swerve modules. 
   *
   * @param steerSetpoints the steer motor setpoints for each swerve module.
   */
  public void setSteerSetpoints(Rotation2d... steerSetpoints) {
    // TODO Throw error?
    if (steerSetpoints.length != 4) return;

    setSetpoints(
        new SwerveModuleState[] {
          new SwerveModuleState(0.0, steerSetpoints[0]),
          new SwerveModuleState(0.0, steerSetpoints[1]),
          new SwerveModuleState(0.0, steerSetpoints[2]),
          new SwerveModuleState(0.0, steerSetpoints[3]),
        },
        true);
  }

  /** Sets the steer motor setpoints to point the swerve modules forwards. */
  public void pointForwards() {
    setSteerSetpoints(
        Rotation2d.fromDegrees(0.0),
        Rotation2d.fromDegrees(0.0),
        Rotation2d.fromDegrees(0.0),
        Rotation2d.fromDegrees(0.0));
  }

  /** Sets the steer motor setpoints to point the swerve modules sideways. */
  public void pointSideways() {
    setSteerSetpoints(
        Rotation2d.fromDegrees(90.0),
        Rotation2d.fromDegrees(90.0),
        Rotation2d.fromDegrees(90.0),
        Rotation2d.fromDegrees(90.0));
  }

  /** Sets the steer motor setpoints to point the swerve modules inwards, like a cross or "X" pattern. */
  public void pointInwards() {
    setSteerSetpoints(
        Rotation2d.fromDegrees(45.0),
        Rotation2d.fromDegrees(-45.0),
        Rotation2d.fromDegrees(45.0),
        Rotation2d.fromDegrees(-45.0));
  }

  /**
   * Sets the swerve's brake mode.
   *
   * @param brake if true, brake.
   */
  public void setBrake(boolean brake) {
    for (SwerveModuleIO swerveModule : swerveModules) {
      swerveModule.setBrake(brake);
    }
  }
}
