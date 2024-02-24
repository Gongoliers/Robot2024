package frc.robot.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.Subsystem;
import frc.lib.Telemetry;
import frc.robot.RobotMechanisms;
import frc.robot.arm.WristMotorIO.WristMotorIOValues;
import frc.robot.arm.ShoulderMotorIO.ShoulderMotorIOValues;
import java.util.function.DoubleSupplier;

/** Subsystem class for the arm subsystem. */
public class Arm extends Subsystem {

  /** Instance variable for the arm subsystem singleton. */
  private static Arm instance = null;

  /** Shoulder motor. */
  private final ShoulderMotorIO shoulderMotor;

  /** Shoulder motor values. */
  private final ShoulderMotorIOValues shoulderMotorValues = new ShoulderMotorIOValues();

  /** Wrist motor. */
  private final WristMotorIO wristMotor;

  /** Wrist motor values. */
  private final WristMotorIOValues wristMotorValues = new WristMotorIOValues();

  private final ArmState initialState = ArmState.STOW;

  private ArmState goal, setpoint;

  /** Creates a new instance of the arm subsystem. */
  private Arm() {
    shoulderMotor = ArmFactory.createShoulderMotor();
    wristMotor = ArmFactory.createWristMotor();

    shoulderMotor.configure();
    wristMotor.configure();

    setPosition(initialState);

    goal = initialState.withWrist(ArmState.STOW.wrist());
    setpoint = getPosition();
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
    wristMotor.update(wristMotorValues);

    setSetpoint(setpoint.nextSetpoint(goal));

    RobotMechanisms.getInstance().setArmState(getPosition());
  }

  @Override
  public void addToShuffleboard(ShuffleboardTab tab) {
    ShuffleboardLayout position = Telemetry.addColumn(tab, "Position");

    position.addDouble(
        "Shoulder Position (deg)",
        () -> Units.rotationsToDegrees(shoulderMotorValues.positionRotations));
    position.addDouble(
        "Wrist Position (deg)", () -> Units.rotationsToDegrees(wristMotorValues.positionRotations));

    ShuffleboardLayout setpoint = Telemetry.addColumn(tab, "Setpoint");

    setpoint.addDouble(
        "Shoulder Setpoint (deg)",
        () -> Units.rotationsToDegrees(getSetpoint().shoulder().position));
    setpoint.addDouble(
        "Wrist Setpoint (deg)", () -> Units.rotationsToDegrees(getSetpoint().wrist().position));
    setpoint.addBoolean("At Setpoint?", this::atSetpoint);

    ShuffleboardLayout goal = Telemetry.addColumn(tab, "Goal");

    goal.addDouble(
        "Shoulder Setpoint (deg)", () -> Units.rotationsToDegrees(getGoal().shoulder().position));
    goal.addDouble(
        "Wrist Setpoint (deg)", () -> Units.rotationsToDegrees(getGoal().wrist().position));
    goal.addBoolean("At Goal?", this::atGoal);

    ShuffleboardLayout voltages = Telemetry.addColumn(tab, "Voltages");

    voltages.addDouble("Shoulder Voltage (V)", () -> shoulderMotorValues.appliedVolts);
    voltages.addDouble("Wrist Voltage (V)", () -> wristMotorValues.appliedVolts);

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

  public void setPosition(ArmState state) {
    shoulderMotor.setPosition(state.shoulder().position);
     wristMotor.setPosition(state.wrist().position);
  }

  /**
   * Gets the position of the arm.
   *
   * @return the position of the arm.
   */
  public ArmState getPosition() {
    shoulderMotor.update(shoulderMotorValues);
    wristMotor.update(wristMotorValues);

    return new ArmState(
        Rotation2d.fromRotations(shoulderMotorValues.positionRotations),
        Rotation2d.fromRotations(wristMotorValues.positionRotations));
  }

  public ArmState getGoal() {
    return goal;
  }

  public boolean atGoal() {
    return atSetpoint() && setpoint.at(goal);
  }

  public void setGoal(ArmState goal) {
    this.goal = goal;
  }

  public ArmState getSetpoint() {
    return setpoint;
  }

  public boolean atSetpoint() {
    return getPosition().at(setpoint);
  }

  private void setSetpoint(ArmState setpoint) {
    this.setpoint = setpoint;

    shoulderMotor.setSetpoint(setpoint.shoulder().position, setpoint.shoulder().velocity);
    wristMotor.setSetpoint(setpoint.wrist().position, setpoint.wrist().velocity);
  }

  public Command to(ArmState goal) {
    return runOnce(() -> setGoal(goal)).andThen(Commands.waitUntil(this::atGoal));
  }

  public Command driveWrist(DoubleSupplier joystick) {
    return run(() -> wristMotor.setVoltage(-joystick.getAsDouble() * 4));
  }
}
