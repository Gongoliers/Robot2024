package frc.robot.arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.Subsystem;
import frc.lib.Telemetry;
import frc.robot.arm.ShoulderMotorIO.ShoulderMotorIOValues;

/** Subsystem class for the arm subsystem. */
public class Arm extends Subsystem {

  /** Instance variable for the arm subsystem singleton. */
  private static Arm instance = null;

  /** Shoulder motor. */
  private final ShoulderMotorIO shoulderMotor;

  /** Shoulder motor values. */
  private final ShoulderMotorIOValues shoulderMotorValues = new ShoulderMotorIOValues();

  /** Creates a new instance of the arm subsystem. */
  private Arm() {
    shoulderMotor = ArmFactory.createShoulderMotor();

    shoulderMotor.configure();
  }

  /**
   * Gets the instance of the arm subsystem.
   *
   * @return the instance of the arm subsystem.
   */
  public static Arm getInstance() {
    if (instance == null) {
      instance = new Arm();
    }

    return instance;
  }

  @Override
  public void periodic() {
    shoulderMotor.update(shoulderMotorValues);
  }

  @Override
  public void addToShuffleboard(ShuffleboardTab tab) {
    ShuffleboardLayout position = Telemetry.addColumn(tab, "Position");

    position.addDouble(
        "Shoulder Position (deg)",
        () -> Units.rotationsToDegrees(shoulderMotorValues.positionRotations));

    ShuffleboardLayout voltages = Telemetry.addColumn(tab, "Voltages");

    voltages.addDouble("Shoulder Voltage (V)", () -> shoulderMotorValues.inputVoltage);

    ShuffleboardLayout derivatives = Telemetry.addColumn(tab, "Derivatives");

    derivatives.addDouble(
        "Shoulder Velocity (rps)", () -> shoulderMotorValues.velocityRotationsPerSecond);
    derivatives.addDouble(
        "Shoulder Acceleration (rpsps)",
        () -> shoulderMotorValues.accelerationRotationsPerSecondPerSecond);
  }

  public State getMeasuredShoulderState() {
    return new TrapezoidProfile.State(
        shoulderMotorValues.positionRotations, shoulderMotorValues.velocityRotationsPerSecond);
  }

  public void setShoulderPosition(double positionRotations) {
    shoulderMotor.setPosition(positionRotations);
  }

  public void setSetpoint(double positionRotations, double velocityRotationsPerSecond) {
    shoulderMotor.setSetpoint(positionRotations, velocityRotationsPerSecond);
  }

  public Command setVoltage(DoubleSupplier volts) {
    return Commands.run(() -> shoulderMotor.setVoltage(volts.getAsDouble()));
  }
}
