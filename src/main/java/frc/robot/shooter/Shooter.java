package frc.robot.shooter;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.Subsystem;
import frc.lib.Telemetry;
import frc.robot.shooter.BeamBreakSensorIO.BeamBreakSensorIOValues;
import frc.robot.shooter.FlywheelMotorIO.FlywheelMotorIOValues;
import frc.robot.shooter.SerializerMotorIO.SerializerMotorIOValues;
import frc.robot.shooter.ShooterConstants.FlywheelConstants;
import frc.robot.shooter.ShooterConstants.SensorConstants;
import frc.robot.shooter.ShooterConstants.SerializerConstants;

/** Subsystem class for the shooter subsystem. */
public class Shooter extends Subsystem {

  /** Instance variable for the shooter subsystem singleton. */
  private static Shooter instance = null;

  /** Beam break sensor. */
  private final BeamBreakSensorIO beamBreakSensor;

  /** Beam break sensor values. */
  private final BeamBreakSensorIOValues beamBreakSensorValues = new BeamBreakSensorIOValues();

  private final Debouncer beamBreakDebouncer =
      new Debouncer(
          SensorConstants.BEAM_BREAK_DEBOUNCE_PERIOD, SensorConstants.BEAM_BREAK_DEBOUNCE_TYPE);

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

    beamBreakDebouncer.calculate(beamBreakSensorValues.isBroken);

    serializerMotor.update(serializerMotorValues);
    flywheelMotor.update(flywheelMotorValues);
  }

  /**
   * Returns true if the shooter is holding a note.
   *
   * @return true if the shooter is holding a note.
   */
  private boolean holdingNote() {
    return beamBreakDebouncer.calculate(beamBreakSensorValues.isBroken);
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
    ShuffleboardLayout sensors = Telemetry.addColumn(tab, "Sensors");

    sensors.addBoolean("Is Beam Break Broken?", () -> beamBreakSensorValues.isBroken);
    sensors.addBoolean("Holding Note?", this::holdingNote);

    ShuffleboardLayout serializer = Telemetry.addColumn(tab, "Serializer");

    serializer.addDouble("Serializer Speed (mps)", this::getSerializerTangentialSpeed);
    serializer.addDouble("Serializer Current (A)", () -> serializerMotorValues.currentAmps);

    ShuffleboardLayout flywheel = Telemetry.addColumn(tab, "Flywheel");

    flywheel.addDouble("Flywheel Speed (mps)", this::getFlywheelTangentialSpeed);
    flywheel.addDouble("Flywheel Current (A)", () -> flywheelMotorValues.currentAmps);
  }

  /**
   * Intakes a note.
   * 
   * @return a command that intakes a note.
   */
  public Command intake() {
    return Commands.parallel(
        Commands.run(() -> serializerMotor.setVoltage(SerializerConstants.INTAKE_VOLTAGE))
            .finallyDo(serializerMotor::stop),
        Commands.run(() -> flywheelMotor.setVoltage(FlywheelConstants.INTAKE_VOLTAGE))
            .finallyDo(flywheelMotor::stop));
  }

  /**
   * Intakes a note until it is held.
   * 
   * @return a command that intakes a note until it is held.
   */
  public Command smartIntake() {
    return intake().until(this::holdingNote).unless(this::holdingNote);
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
   * Serializes a note until it is not held.
   * 
   * @return a command that serializes a note until it is not held.
   */
  public Command smartSerialize() {
    return serialize().onlyWhile(this::holdingNote).onlyIf(this::holdingNote);
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
  public Command shoot() {
    return Commands.deadline(
        Commands.waitSeconds(SerializerConstants.SHOOT_DELAY).andThen(serialize()), spin());
  }

  /**
   * Shoots a note by spinning the flywheel then serializing a note until it is not held.
   * 
   * @return a command that shoots a note until it is not held.
   */
  public Command smartShoot() {
    return Commands.deadline(
        Commands.waitSeconds(SerializerConstants.SHOOT_DELAY).andThen(smartSerialize()),
        spin().onlyIf(this::holdingNote));
  }
}
