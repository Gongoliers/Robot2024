package frc.robot.odometry;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.lib.Subsystem;
import frc.lib.Telemetry;
import frc.robot.odometry.GyroscopeIO.GyroscopeIOValues;
import frc.robot.swerve.Swerve;
import java.util.function.Supplier;

/** Subsystem class for the odometry subsystem. */
public class Odometry extends Subsystem {

  /** Instance variable for the odometry subsystem singleton. */
  private static Odometry instance = null;

  /** Gyroscope. */
  private final GyroscopeIO gyroscope;

  /** Gyroscope values. */
  private final GyroscopeIOValues gyroscopeValues = new GyroscopeIOValues();

  /** Supplies swerve module positions. */
  private final Supplier<SwerveModulePosition[]> swerveModulePositionsSupplier;

  /** Supplies swerve velocity. */
  private final Supplier<ChassisSpeeds> swerveChassisSpeedsSupplier;

  /** Pose estimator using the swerve drive. */
  private final SwerveDrivePoseEstimator swervePoseEstimator;

  /** Creates a new instance of the odometry subsystem. */
  private Odometry() {
    gyroscope = OdometryFactory.createGyroscope();
    gyroscope.configure();

    swerveModulePositionsSupplier = () -> Swerve.getInstance().getModulePositions();

    swerveChassisSpeedsSupplier = () -> Swerve.getInstance().getChassisSpeeds();

    gyroscope.update(gyroscopeValues);

    Rotation2d initialGyroRotation = Rotation2d.fromRotations(gyroscopeValues.yawRotations);

    Pose2d initialPose = new Pose2d();

    swervePoseEstimator =
        new SwerveDrivePoseEstimator(
            Swerve.getInstance().getKinematics(),
            initialGyroRotation,
            swerveModulePositionsSupplier.get(),
            initialPose);
  }

  /**
   * Gets the instance of the odometry subsystem.
   *
   * @return the instance of the odometry subsystem.
   */
  public static Odometry getInstance() {
    if (instance == null) {
      instance = new Odometry();
    }

    return instance;
  }

  @Override
  public void periodic() {
    gyroscope.update(gyroscopeValues);

    swervePoseEstimator.update(
        Rotation2d.fromRotations(gyroscopeValues.yawRotations),
        swerveModulePositionsSupplier.get());
  }

  @Override
  public void addToShuffleboard(ShuffleboardTab tab) {
    ShuffleboardLayout position = Telemetry.addColumn(tab, "Position");

    position.addDouble("X (m)", () -> getPosition().getX());
    position.addDouble("Y (m)", () -> getPosition().getY());
    position.addDouble("Rotation (deg)", () -> getPosition().getRotation().getDegrees());

    ShuffleboardLayout velocity = Telemetry.addColumn(tab, "Velocity");

    velocity.addDouble("X Velocity (mps)", () -> getVelocity().getX());
    velocity.addDouble("Y Velocity (mps)", () -> getVelocity().getY());
    velocity.addDouble(
        "Translation Velocity (mps)", () -> getVelocity().getTranslation().getNorm());
    velocity.addDouble("Rotation Velocity (dps)", () -> getVelocity().getRotation().getDegrees());
  }

  /**
   * Gets the position of the robot on the field.
   *
   * @return the position of the robot on the field.
   */
  public Pose2d getPosition() {
    return swervePoseEstimator.getEstimatedPosition();
  }

  /**
   * Gets the velocity of the robot on the field.
   *
   * @return the velocity of the robot on the field.
   */
  public Transform2d getVelocity() {
    ChassisSpeeds chassisSpeeds = swerveChassisSpeedsSupplier.get();

    Rotation2d rotation = getPosition().getRotation();

    double xVelocityMetersPerSecond =
        chassisSpeeds.vxMetersPerSecond * rotation.getCos()
            - chassisSpeeds.vyMetersPerSecond * rotation.getSin();
    double yVelocityMetersPerSecond =
        chassisSpeeds.vxMetersPerSecond * rotation.getSin()
            + chassisSpeeds.vyMetersPerSecond * rotation.getCos();

    Translation2d translationVelocity =
        new Translation2d(xVelocityMetersPerSecond, yVelocityMetersPerSecond);

    Rotation2d rotationVelocity = Rotation2d.fromRadians(chassisSpeeds.omegaRadiansPerSecond);

    return new Transform2d(translationVelocity, rotationVelocity);
  }
}
