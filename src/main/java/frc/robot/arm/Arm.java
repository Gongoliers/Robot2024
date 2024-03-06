package frc.robot.arm;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.Subsystem;
import frc.lib.Telemetry;
import frc.lib.TrapezoidProfileTelemetry;
import frc.robot.RobotMechanisms;
import frc.robot.arm.LimitSwitchIO.LimitSwitchIOValues;
import frc.robot.arm.ShoulderMotorIO.ShoulderMotorIOValues;
import frc.robot.arm.WristMotorIO.WristMotorIOValues;

/** Subsystem class for the arm subsystem. */
public class Arm extends Subsystem {

  /** Instance variable for the arm subsystem singleton. */
  private static Arm instance = null;

  private final RobotMechanisms mechanism;

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

  /** Arm goal and setpoint states. */
  private ArmState goal, setpoint;

  /** Telemetry for the shoulder and wrist trapezoid profiles. */
  private final TrapezoidProfileTelemetry shoulderProfileTelemetry, wristProfileTelemetry;

  /** Creates a new instance of the arm subsystem. */
  private Arm() {
    mechanism = RobotMechanisms.getInstance();

    limitSwitch = ArmFactory.createLimitSwitch();
    shoulderMotor = ArmFactory.createShoulderMotor();
    wristMotor = ArmFactory.createWristMotor();

    limitSwitch.configure();
    shoulderMotor.configure();
    wristMotor.configure();

    setPosition(ArmState.INIT);
    clearProfile();

    shoulderProfileTelemetry = new TrapezoidProfileTelemetry("arm/shoulderProfile");
    wristProfileTelemetry = new TrapezoidProfileTelemetry("arm/wristProfile");
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

    mechanism.updateArm(getMeasuredState());

    shoulderProfileTelemetry.update(
        getMeasuredState().shoulder(), getSetpoint().shoulder(), getGoal().shoulder());
    wristProfileTelemetry.update(
        getMeasuredState().wrist(), getSetpoint().wrist(), getGoal().wrist());
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

    ShuffleboardLayout goal = Telemetry.addColumn(tab, "Goal");

    goal.addDouble(
        "Shoulder Setpoint (deg)", () -> Units.rotationsToDegrees(getGoal().shoulder().position));
    goal.addDouble(
        "Wrist Setpoint (deg)", () -> Units.rotationsToDegrees(getGoal().wrist().position));

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

  /**
   * Sets the position of the arm to the supplied state.
   *
   * @param state the state containing the position of the arm.
   */
  public void setPosition(ArmState state) {
    shoulderMotor.setPosition(state.shoulder().position);
    wristMotor.setPosition(state.wrist().position);
  }

  /**
   * Resets the goal and setpoints of the arm to the arm's current position. Commands the arm to
   * hold its current position.
   */
  public void clearProfile() {
    ArmState position = getMeasuredState().position();

    goal = position;
    setpoint = position;
  }

  /**
   * Returns the arm's measured state.
   *
   * @return the arm's measured state.
   */
  public ArmState getMeasuredState() {
    shoulderMotor.update(shoulderMotorValues);
    wristMotor.update(wristMotorValues);

    TrapezoidProfile.State measuredShoulderState =
        new TrapezoidProfile.State(
            shoulderMotorValues.positionRotations, shoulderMotorValues.velocityRotationsPerSecond);
    TrapezoidProfile.State measuredWristState =
        new TrapezoidProfile.State(
            wristMotorValues.positionRotations, wristMotorValues.velocityRotationsPerSecond);

    return new ArmState(measuredShoulderState, measuredWristState);
  }

  /**
   * Returns true if the arm is at a given state.
   *
   * @param state the state to compare to.
   * @return true if the arm is at a given state.
   */
  public boolean at(ArmState state) {
    return getMeasuredState().at(state);
  }

  /**
   * Returns the arm's goal.
   *
   * @return the arm's goal.
   */
  public ArmState getGoal() {
    return this.goal;
  }

  /**
   * Sets the arm's goal.
   *
   * <p>Calling this method does not alter the arm's motion; it simply updates a value used for
   * telemetry.
   *
   * @param goal the arm's goal.
   */
  public void setGoal(ArmState goal) {
    this.goal = goal;
  }

  /**
   * Returns the arm's setpoint.
   *
   * @return the arm's setpoint.
   */
  public ArmState getSetpoint() {
    return this.setpoint;
  }

  /**
   * Sets the arm's setpoint.
   *
   * <p>Calling this method does not alter the arm's motion; it simply updates a value used for
   * telemetry.
   *
   * @param setpoint the arm's setpoint.
   */
  public void setSetpoint(ArmState setpoint) {
    this.setpoint = setpoint;
  }

  /**
   * Applies a setpoint to the arm's controllers.
   *
   * <p>Calling this method alters the arm's motion.
   *
   * @param setpoint the arm's setpoint.
   */
  public void applySetpoint(ArmState setpoint) {
    shoulderMotor.setSetpoint(setpoint.shoulder().position, setpoint.shoulder().velocity);
    wristMotor.setSetpoint(setpoint.wrist().position, setpoint.wrist().velocity);
  }

  /**
   * Returns a command that moves the shoulder to a goal's shoulder position.
   *
   * @param goal the goal position to move to.
   * @return a command that moves the shoulder to a goal's shoulder position.
   */
  private Command shoulderTo(ArmState goal) {
    return new MoveShoulderCommand(goal);
  }

  /**
   * Returns a command that moves the wrist to a goal's wrist position.
   *
   * @param goal the goal position to move to.
   * @return a command that moves the wrist to a goal's wrist position.
   */
  public Command wristTo(ArmState goal) {
    return new MoveWristCommand(goal);
  }

  /**
   * Returns a command that moves the arm to the amp position.
   *
   * @return a comamnd that moves the arm to the amp position.
   */
  public Command amp() {
    return shoulderTo(ArmState.AMP).andThen(wristTo(ArmState.AMP));
  }

  /**
   * Returns a command that moves the arm to the stow position.
   *
   * @return a command that moves the arm to the stow position.
   */
  public Command stow() {
    return wristTo(ArmState.STOW).andThen(shoulderTo(ArmState.STOW));
  }

  /**
   * Returns a command that moves the arm to the stow position and resets the position of the arm.
   *
   * <p>When the limit switch detects that the arm is in the stow position, the arm position is
   * reset to be equal to the stow position.
   *
   * @return a command that moves the arm to the stow position and resets the position of the arm.
   */
  public Command home() {
    return wristTo(ArmState.STOW)
        .andThen(shoulderTo(ArmState.STOW).until(() -> limitSwitchValues.isPressed))
        .finallyDo(
            interrupted -> {
              if (!interrupted) {
                setPosition(ArmState.STOW);
              }
            });
  }
}
