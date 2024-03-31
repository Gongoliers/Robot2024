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

    serializer.addDouble(
        "Serializer Speed (rps)", () -> serializerMotorValues.velocityRotationsPerSecond);
    serializer.addDouble("Serializer Current (A)", () -> serializerMotorValues.currentAmps);

    ShuffleboardLayout flywheel = Telemetry.addColumn(tab, "Flywheel");

    flywheel.addDouble(
        "Flywheel Speed (rps)", () -> flywheelMotorValues.velocityRotationsPerSecond);
    flywheel.addDouble("Flywheel Current (A)", () -> flywheelMotorValues.currentAmps);
  }

  public double getFlywheelVelocity() {
    flywheelMotor.update(flywheelMotorValues);

    return flywheelMotorValues.velocityRotationsPerSecond;
  }

  public double getSerializerVelocity() {
    serializerMotor.update(serializerMotorValues);

    return serializerMotorValues.velocityRotationsPerSecond;
  }

  /**
   * Intakes a note.
   *
   * @return a command that intakes a note.
   */
  public Command intake() {
    return Commands.runEnd(
        () -> serializerMotor.setSetpoint(SerializerConstants.INTAKE_VELOCITY),
        () -> serializerMotor.setSetpoint(0.0));
  }

  /**
   * Serializes a note.
   *
   * @return a command that serializes a note.
   */
  public Command serialize() {
    return Commands.runEnd(
        () -> serializerMotor.setSetpoint(SerializerConstants.SERIALIZE_VELOCITY),
        () -> serializerMotor.setSetpoint(0.0));
  }

  /**
   * Spins the flywheel.
   *
   * @return a command that spins the flywheel.
   */
  public Command spin() {
    return Commands.runEnd(
        () -> flywheelMotor.setSetpoint(FlywheelConstants.SHOOT_VELOCITY),
        () -> flywheelMotor.setSetpoint(0.0));
  }
}
