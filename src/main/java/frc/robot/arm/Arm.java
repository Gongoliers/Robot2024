package frc.robot.arm;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.lib.Subsystem;
import frc.lib.Telemetry;
import frc.robot.arm.LimitSwitchIO.LimitSwitchIOValues;
import frc.robot.arm.ShoulderMotorIO.ShoulderMotorIOValues;
import frc.robot.arm.WristMotorIO.WristMotorIOValues;

/** Subsystem class for the arm subsystem. */
public class Arm extends Subsystem {

  /** Instance variable for the arm subsystem singleton. */
  private static Arm instance = null;

  /** Limit switch. */
  private final LimitSwitchIO limitSwitch;

  /** Limit switch values. */
  private final LimitSwitchIOValues limitSwitchValues = new LimitSwitchIOValues();

  /** Shoulder motor. */
  private final ShoulderMotorIO shoulderMotor;

  /** Shoulder motor values. */
  private final ShoulderMotorIOValues shoulderMotorValues = new ShoulderMotorIOValues();

  /** Wrist motor. */
  private final WristMotorIO wristMotor;

  /** Wrist motor values. */
  private final WristMotorIOValues wristMotorValues = new WristMotorIOValues();

  /** Creates a new instance of the arm subsystem. */
  private Arm() {
    limitSwitch = ArmFactory.createLimitSwitch();
    shoulderMotor = ArmFactory.createShoulderMotor();
    wristMotor = ArmFactory.createWristMotor();

    limitSwitch.configure();
    shoulderMotor.configure();
    wristMotor.configure();

    // setPosition(ArmState.INIT);
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
    limitSwitch.update(limitSwitchValues);
    shoulderMotor.update(shoulderMotorValues);
    wristMotor.update(wristMotorValues);
  }

  @Override
  public void addToShuffleboard(ShuffleboardTab tab) {
    ShuffleboardLayout limitSwitch = Telemetry.addColumn(tab, "Limit Switch");

    limitSwitch.addBoolean("Is Pressed?", () -> limitSwitchValues.isPressed);

    ShuffleboardLayout position = Telemetry.addColumn(tab, "Position");

    position.addDouble(
        "Shoulder Position (deg)",
        () -> Units.rotationsToDegrees(shoulderMotorValues.positionRotations));
    position.addDouble(
        "Wrist Position (deg)", () -> Units.rotationsToDegrees(wristMotorValues.positionRotations));

    ShuffleboardLayout voltages = Telemetry.addColumn(tab, "Voltages");

    voltages.addDouble("Shoulder Voltage (V)", () -> shoulderMotorValues.inputVoltage);
    voltages.addDouble("Wrist Voltage (V)", () -> wristMotorValues.inputVoltage);

    ShuffleboardLayout derivatives = Telemetry.addColumn(tab, "Derivatives");

    derivatives.addDouble(
        "Shoulder Velocity (rps)", () -> shoulderMotorValues.velocityRotationsPerSecond);
    derivatives.addDouble(
        "Shoulder Acceleration (rpsps)",
        () -> shoulderMotorValues.accelerationRotationsPerSecondPerSecond);
    derivatives.addDouble(
        "Wrist Velocity (rps)", () -> wristMotorValues.velocityRotationsPerSecond);
    derivatives.addDouble(
        "Wrist Acceleration (rpsps)",
        () -> wristMotorValues.accelerationRotationsPerSecondPerSecond);
  }

  public State getMeasuredShoulderState() {
    return new TrapezoidProfile.State(
        shoulderMotorValues.positionRotations, shoulderMotorValues.velocityRotationsPerSecond);
  }

  public State getMeasuredWristState() {
    return new TrapezoidProfile.State(
        wristMotorValues.positionRotations, wristMotorValues.velocityRotationsPerSecond);
  }
}
