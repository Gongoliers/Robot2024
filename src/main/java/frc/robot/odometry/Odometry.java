package frc.robot.odometry;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.AllianceFlipHelper;
import frc.lib.Subsystem;
import frc.lib.Telemetry;
import frc.robot.odometry.GyroscopeIO.GyroscopeIOValues;
import frc.robot.swerve.Swerve;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
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

  /** Field. */
  private final Field2d field;

  /** List of functions to be called when pose is manually updated. */
  private final List<Consumer<Pose2d>> poseUpdateConsumers;

  /** Creates a new instance of the odometry subsystem. */
  private Odometry() {
    gyroscope = OdometryFactory.createGyroscope(this);
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

    field = new Field2d();

    poseUpdateConsumers = new ArrayList<Consumer<Pose2d>>();
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

    field.setRobotPose(getPosition());
  }

  @Override
  public void addToShuffleboard(ShuffleboardTab tab) {
    ShuffleboardLayout position = Telemetry.addColumn(tab, "Position");

    position.addDouble("X (m)", () -> getPosition().getX());
    position.addDouble("Y (m)", () -> getPosition().getY());
    position.addDouble("Rotation (deg)", () -> getPosition().getRotation().getDegrees());

    ShuffleboardLayout velocity = Telemetry.addColumn(tab, "Velocity");

    velocity.addDouble("X Velocity (mps)", () -> getVelocity().dx);
    velocity.addDouble("Y Velocity (mps)", () -> getVelocity().dy);
    velocity.addDouble(
        "Rotation Velocity (dps)", () -> Units.radiansToDegrees((getVelocity().dtheta)));

    ShuffleboardLayout field = Telemetry.addColumn(tab, "Field");

    field.add("Field", this.field);
  }

  /**
   * Returns the position of the robot on the field.
   *
   * @return the position of the robot on the field.
   */
  public Pose2d getPosition() {
    return swervePoseEstimator.getEstimatedPosition();
  }

  /**
   * Returns the rotation of the robot on the field where zero is away from the blue alliance wall.
   *
   * @return the rotation of the robot on the field where zero is away from the blue alliance wall.
   */
  public Rotation2d getFieldRelativeHeading() {
    return getPosition().getRotation();
  }

  /**
   * Returns the rotation of the robot on the field where zero is away from the driver regardless of
   * alliance.
   *
   * @return the rotation of the robot on the field where zero is away from the driver regardless of
   *     alliance.
   */
  public Rotation2d getDriverRelativeHeading() {
    Rotation2d fieldRelativeHeading = getFieldRelativeHeading();

    if (AllianceFlipHelper.shouldFlip()) {
      return fieldRelativeHeading.plus(Rotation2d.fromDegrees(180));
    }

    return fieldRelativeHeading;
  }

  /**
   * Sets the position of the robot on the field.
   *
   * @param position the position of the robot on the field.
   */
  public void setPosition(Pose2d position) {
    swervePoseEstimator.resetPosition(
        Rotation2d.fromRotations(gyroscopeValues.yawRotations),
        swerveModulePositionsSupplier.get(),
        position);
  }

  /**
   * Sets the rotation of the robot on the field.
   *
   * @param rotation the rotation of the robot on the field.
   */
  public void setRotation(Rotation2d rotation) {
    Pose2d position = getPosition();

    setPosition(new Pose2d(position.getTranslation(), rotation));
  }

  /**
   * Adds a consumer for when pose is manually updated.
   *
   * @param consumer consumer for when pose is manually updated.
   */
  public void onPoseUpdate(Consumer<Pose2d> consumer) {
    poseUpdateConsumers.add(consumer);
  }

  /**
   * Tares the rotation of the robot.
   *
   * @return a command that zeroes the rotation of the robot.
   */
  public Command tare() {
    return Commands.runOnce(
        () -> {
          if (AllianceFlipHelper.shouldFlip()) {
            setRotation(Rotation2d.fromDegrees(180));
          } else {
            setRotation(Rotation2d.fromDegrees(0));
          }

          poseUpdateConsumers.forEach(consumer -> consumer.accept(getPosition()));
        });
  }

  /**
   * Gets the velocity of the robot on the field.
   *
   * @return the velocity of the robot on the field.
   */
  public Twist2d getVelocity() {
    // Guards against calling before the swerve pose estimator is
    // intialized
    if (swervePoseEstimator == null) return new Twist2d();

    ChassisSpeeds chassisSpeeds = swerveChassisSpeedsSupplier.get();

    Rotation2d rotation = getPosition().getRotation();

    double xVelocityMetersPerSecond =
        chassisSpeeds.vxMetersPerSecond * rotation.getCos()
            - chassisSpeeds.vyMetersPerSecond * rotation.getSin();
    double yVelocityMetersPerSecond =
        chassisSpeeds.vxMetersPerSecond * rotation.getSin()
            + chassisSpeeds.vyMetersPerSecond * rotation.getCos();

    return new Twist2d(
        xVelocityMetersPerSecond, yVelocityMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond);
  }
}
