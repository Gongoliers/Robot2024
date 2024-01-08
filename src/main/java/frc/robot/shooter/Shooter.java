package frc.robot.shooter;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.lib.Subsystem;
import frc.robot.shooter.BeamBreakSensorIO.BeamBreakSensorIOValues;
import frc.robot.shooter.FlywheelMotorIO.FlywheelMotorIOValues;
import frc.robot.shooter.SerializerMotorIO.SerializerMotorIOValues;
import frc.robot.shooter.ShooterConstants.FlywheelConstants;
import frc.robot.shooter.ShooterConstants.SerializerConstants;

/** Subsystem class for the shooter subsystem. */
public class Shooter extends Subsystem {

  /** Instance variable for the shooter subsystem singleton. */
  private static Shooter instance = null;

  /** Beam break sensor. */
  private final BeamBreakSensorIO beamBreakSensor;

  /** Beam break sensor values. */
  private final BeamBreakSensorIOValues beamBreakSensorValues = new BeamBreakSensorIOValues();

  /** Serializer motor. */
  private final SerializerMotorIO serializerMotor;

  /** Serializer motor values. */
  private final SerializerMotorIOValues serializerMotorValues = new SerializerMotorIOValues();

  /** Flywheel motor. */
  private final FlywheelMotorIO flywheelMotor;

  /** Flywheel motor values. */
  private final FlywheelMotorIOValues flywheelMotorValues = new FlywheelMotorIOValues();

  /** Creates a new instance of the shooter subsystem. */
  private Shooter() {
    beamBreakSensor = ShooterFactory.createBeamBreakSensor();
    serializerMotor = ShooterFactory.createSerializerMotor();
    flywheelMotor = ShooterFactory.createFlywheelMotor();
  }

  /**
   * Gets the instance of the shooter subsystem.
   *
   * @return the instance of the shooter subsystem.
   */
  public static Shooter getInstance() {
    if (instance == null) {
      instance = new Shooter();
    }

    return instance;
  }

  @Override
  public void periodic() {
    beamBreakSensor.update(beamBreakSensorValues);
    serializerMotor.update(serializerMotorValues);
    flywheelMotor.update(flywheelMotorValues);
  }

  /**
   * Calculates the tangential speed of the serializer in meters per second.
   *
   * @return the tangential speed of the serializer in meters per second.
   */
  private double getSerializerTangentialSpeed() {
    return serializerMotorValues.angularVelocityRotationsPerSecond * SerializerConstants.RADIUS;
  }

  /**
   * Calculates the tangential speed of the flywheel in meters per second.
   *
   * @return the tangential speed of the flywheel in meters per second.
   */
  private double getFlywheelTangentialSpeed() {
    return flywheelMotorValues.angularVelocityRotationsPerSecond * FlywheelConstants.RADIUS;
  }

  @Override
  public void addToShuffleboard(ShuffleboardTab tab) {
    ShuffleboardLayout sensors =
        tab.getLayout("Sensors", BuiltInLayouts.kList).withPosition(0, 0).withSize(2, 4);

    sensors.addBoolean("Beam Break Sensor Is Broken?", () -> beamBreakSensorValues.isBroken);

    ShuffleboardLayout serializer =
        tab.getLayout("Serializer", BuiltInLayouts.kList).withPosition(2, 0).withSize(2, 4);

    serializer.addDouble("Serializer Speed (mps)", this::getSerializerTangentialSpeed);
    serializer.addDouble("Serializer Current (A)", () -> serializerMotorValues.currentAmps);

    ShuffleboardLayout flywheel =
        tab.getLayout("Flywheel", BuiltInLayouts.kList).withPosition(4, 0).withSize(2, 4);

    flywheel.addDouble("Flywheel Speed (mps)", this::getFlywheelTangentialSpeed);
    flywheel.addDouble("Flywheel Current (A)", () -> flywheelMotorValues.currentAmps);
  }
}
