package frc.robot.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.Subsystem;
import frc.lib.Telemetry;
import frc.robot.RobotMechanisms;
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

  private ArmState goal, setpoint;

  /** Creates a new instance of the arm subsystem. */
  private Arm() {
    limitSwitch = ArmFactory.createLimitSwitch();
    shoulderMotor = ArmFactory.createShoulderMotor();
    wristMotor = ArmFactory.createWristMotor();

    limitSwitch.configure();
    shoulderMotor.configure();
    wristMotor.configure();

    ArmState initialState = ArmState.STOW.withShoulder(Rotation2d.fromDegrees(45));

    setPosition(initialState);

    // Since setPosition also resets goal and setpoint, this is redundant, but will protect from nullish errors
    goal = initialState;
    setpoint = initialState;
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

    if (limitSwitchValues.isPressed) {
      setPosition(getPosition().withShoulder(ArmState.STOW.shoulder()));
    }

    setSetpoint(setpoint.nextSetpoint(goal));

    RobotMechanisms.getInstance().setArmState(getPosition());
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

  public void setPosition(ArmState state) {
    shoulderMotor.setPosition(state.shoulder().position);
    wristMotor.setPosition(state.wrist().position);

    goal = state;
    setpoint = state;
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

  public Command home() {
    return run(() -> shoulderMotor.setVoltage(-1)).until(() -> limitSwitchValues.isPressed);
  }

  private MoveShoulderCommand moveShoulder(ArmState goal) {
    return new MoveShoulderCommand(goal);
  }

  private MoveWristCommand moveWrist(ArmState goal) {
    return new MoveWristCommand(goal);
  }

  public Command moveShoulderThenWrist(ArmState goal) {
    return moveShoulder(goal).andThen(moveWrist(goal));
  }

  public Command moveWristThenShoulder(ArmState goal) {
    return moveWrist(goal).andThen(moveShoulder(goal));
  }

  public Command stowFromUp() {
    return moveWristThenShoulder(ArmState.STOW);
  }

  public Command amp() {
    return moveShoulderThenWrist(ArmState.AMP);
  }
}
