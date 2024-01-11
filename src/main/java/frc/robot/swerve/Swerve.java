package frc.robot.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
    for (int i = 0; i < 4; i++) {
      SwerveModuleIO swerveModule = swerveModules[i];

      ShuffleboardLayout swerveModuleColumn = Telemetry.addColumn(tab, "Module " + i);

      swerveModuleColumn.addDouble("Angle (deg)", () -> swerveModule.getState().angle.getDegrees());
      swerveModuleColumn.addDouble(
          "Velocity (mps)", () -> swerveModule.getState().speedMetersPerSecond);
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
   * Orients all swerve modules.
   *
   * @param orientations orientations for each swerve modules.
   * @return a command that orients all swerve modules.
   */
  public Command orientModules(Rotation2d[] orientations) {
    return Commands.run(
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
}
