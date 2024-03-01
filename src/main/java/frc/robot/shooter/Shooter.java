package frc.robot.shooter;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.Subsystem;
import frc.lib.Telemetry;
import frc.robot.shooter.FlywheelMotorIO.FlywheelMotorIOValues;
import frc.robot.shooter.SerializerMotorIO.SerializerMotorIOValues;
import frc.robot.shooter.ShooterConstants.FlywheelConstants;
import frc.robot.shooter.ShooterConstants.SerializerConstants;
import frc.robot.shooter.ShooterConstants.ShooterCommandConstants;

/** Subsystem class for the shooter subsystem. */
public class Shooter extends Subsystem {

  /** Instance variable for the shooter subsystem singleton. */
  private static Shooter instance = null;

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
    serializerMotor.update(serializerMotorValues);
    flywheelMotor.update(flywheelMotorValues);
  }

  @Override
  public void addToShuffleboard(ShuffleboardTab tab) {
    ShuffleboardLayout serializer = Telemetry.addColumn(tab, "Serializer");

    serializer.addDouble("Serializer Speed (mps)", this::getSerializerTangentialSpeed);
    serializer.addDouble("Serializer Current (A)", () -> serializerMotorValues.currentAmps);

    ShuffleboardLayout flywheel = Telemetry.addColumn(tab, "Flywheel");

    flywheel.addDouble("Flywheel Speed (mps)", this::getFlywheelTangentialSpeed);
    flywheel.addDouble("Flywheel Current (A)", () -> flywheelMotorValues.currentAmps);
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

  /**
   * Intakes a note.
   *
   * @return a command that intakes a note.
   */
  public Command intake() {
    return Commands.run(() -> serializerMotor.setVoltage(SerializerConstants.INTAKE_VOLTAGE))
        .finallyDo(serializerMotor::stop);
  }

  /**
   * Serializes a note.
   *
   * @return a command that serializes a note.
   */
  public Command serialize() {
    return Commands.run(() -> serializerMotor.setVoltage(SerializerConstants.SERIALIZE_VOLTAGE))
        .finallyDo(serializerMotor::stop);
  }

  /**
   * Spins the flywheel.
   *
   * @return a command that spins the flywheel.
   */
  public Command spin() {
    return Commands.run(() -> flywheelMotor.setVoltage(FlywheelConstants.SHOOT_VOLTAGE))
        .finallyDo(flywheelMotor::stop);
  }

  /**
   * Shoots a note by spinning the flywheel then serializing a note.
   *
   * @return a command that shoots a note.
   */
  public Command autoShoot() {
    return Commands.deadline(
        Commands.waitSeconds(ShooterCommandConstants.PRE_SHOOT_DELAY).andThen(serialize()), spin());
  }
}
