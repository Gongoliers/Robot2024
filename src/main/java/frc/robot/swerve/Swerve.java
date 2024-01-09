package frc.robot.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
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

  /** Swerve kinematics.  */
  private final SwerveDriveKinematics swerveKinematics;

  /** Creates a new instance of the swerve subsystem. */
  private Swerve() {
    swerveModules[0] = SwerveFactory.createModule(SwerveConstants.NORTH_WEST_MODULE_CONFIG);
    swerveModules[1] = SwerveFactory.createModule(SwerveConstants.NORTH_EAST_MODULE_CONFIG);
    swerveModules[2] = SwerveFactory.createModule(SwerveConstants.SOUTH_EAST_MODULE_CONFIG);
    swerveModules[3] = SwerveFactory.createModule(SwerveConstants.SOUTH_WEST_MODULE_CONFIG);

    swerveKinematics = new SwerveDriveKinematics(SwerveConstants.NORTH_WEST_MODULE_CONFIG.position(), SwerveConstants.NORTH_EAST_MODULE_CONFIG.position(), SwerveConstants.SOUTH_EAST_MODULE_CONFIG.position(), SwerveConstants.SOUTH_WEST_MODULE_CONFIG.position());
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
    ShuffleboardLayout northWestModule = Telemetry.addColumn(tab, "North West Module");

    northWestModule.addDouble("Angle (deg)", () -> swerveModules[0].getPosition().angle.getDegrees());
    northWestModule.addDouble("Distance (m)", () -> swerveModules[0].getPosition().distanceMeters);

    ShuffleboardLayout northEastModule = Telemetry.addColumn(tab, "North East Module");

    northEastModule.addDouble("Angle (deg)", () -> swerveModules[1].getPosition().angle.getDegrees());
    northEastModule.addDouble("Distance (m)", () -> swerveModules[1].getPosition().distanceMeters);

    ShuffleboardLayout southEastModule = Telemetry.addColumn(tab, "South East Module");

    southEastModule.addDouble("Angle (deg)", () -> swerveModules[2].getPosition().angle.getDegrees());
    southEastModule.addDouble("Distance (m)", () -> swerveModules[2].getPosition().distanceMeters);

    ShuffleboardLayout southWestModule = Telemetry.addColumn(tab, "South West Module");

    southWestModule.addDouble("Angle (deg)", () -> swerveModules[3].getPosition().angle.getDegrees());
    southWestModule.addDouble("Distance (m)", () -> swerveModules[3].getPosition().distanceMeters);
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
    return Commands.run(() -> {
      setSetpoints(new SwerveModuleState[] {
        new SwerveModuleState(0.0, orientations[0]),
        new SwerveModuleState(0.0, orientations[1]),
        new SwerveModuleState(0.0, orientations[2]),
        new SwerveModuleState(0.0, orientations[3]),
      }, false);
    });
  }

  public Command forwards() {
    return orientModules(new Rotation2d[] {
      Rotation2d.fromDegrees(0.0),
      Rotation2d.fromDegrees(0.0),
      Rotation2d.fromDegrees(0.0),
      Rotation2d.fromDegrees(0.0)
    });
  }

  public Command sideways() {
    return orientModules(new Rotation2d[] {
      Rotation2d.fromDegrees(90.0),
      Rotation2d.fromDegrees(90.0),
      Rotation2d.fromDegrees(90.0),
      Rotation2d.fromDegrees(90.0)
    });
  }
}
