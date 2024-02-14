package frc.robot.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.Subsystem;
import frc.lib.Telemetry;
import frc.lib.TwoJointedArmFeedforward;
import frc.robot.RobotMechanisms;
import frc.robot.arm.ArmConstants.ElbowMotorConstants;
import frc.robot.arm.ArmConstants.ShoulderMotorConstants;
import frc.robot.arm.ElbowMotorIO.ElbowMotorIOValues;
import frc.robot.arm.ShoulderMotorIO.ShoulderMotorIOValues;

/** Subsystem class for the arm subsystem. */
public class Arm extends Subsystem {

  /** Instance variable for the arm subsystem singleton. */
  private static Arm instance = null;

  /** Shoulder motor. */
  private final ShoulderMotorIO shoulderMotor;

  /** Shoulder motor values. */
  private final ShoulderMotorIOValues shoulderMotorValues = new ShoulderMotorIOValues();

  /** Elbow motor. */
  private final ElbowMotorIO elbowMotor;

  /** Elbow motor values. */
  private final ElbowMotorIOValues elbowMotorValues = new ElbowMotorIOValues();

  private ArmState goal, setpoint;

  private final TwoJointedArmFeedforward feedforward;

  /** Creates a new instance of the arm subsystem. */
  private Arm() {
    shoulderMotor = ArmFactory.createShoulderMotor();
    elbowMotor = ArmFactory.createElbowMotor();

    setPosition(ArmState.STOW);

    goal = getPosition();
    setpoint = getPosition();

    feedforward =
        new TwoJointedArmFeedforward(
            ShoulderMotorConstants.JOINT_CONSTANTS, ElbowMotorConstants.JOINT_CONSTANTS);
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

    setSetpoint(getSetpoint().nextSetpoint(goal));

    RobotMechanisms.getInstance().setArmState(getPosition());
  }

  @Override
  public void addToShuffleboard(ShuffleboardTab tab) {
    ShuffleboardLayout position = Telemetry.addColumn(tab, "Position");

    position.addDouble(
        "Shoulder Position (deg)",
        () -> Units.rotationsToDegrees(shoulderMotorValues.positionRotations));
    position.addDouble(
        "Elbow Position (deg)", () -> Units.rotationsToDegrees(elbowMotorValues.positionRotations));

    ShuffleboardLayout setpoint = Telemetry.addColumn(tab, "Setpoint");

    setpoint.addDouble(
        "Shoulder Setpoint (deg)",
        () -> Units.rotationsToDegrees(getSetpoint().shoulder().position));
    setpoint.addDouble(
        "Elbow Setpoint (deg)", () -> Units.rotationsToDegrees(getSetpoint().elbow().position));
    setpoint.addDouble(
        "Wrist Setpoint (deg)", () -> Units.rotationsToDegrees(getSetpoint().wrist().position));
    setpoint.addBoolean("At Setpoint?", this::atSetpoint);

    ShuffleboardLayout goal = Telemetry.addColumn(tab, "Goal");

    goal.addDouble(
        "Shoulder Setpoint (deg)", () -> Units.rotationsToDegrees(getGoal().shoulder().position));
    goal.addDouble(
        "Elbow Setpoint (deg)", () -> Units.rotationsToDegrees(getGoal().elbow().position));
    goal.addDouble(
        "Wrist Setpoint (deg)", () -> Units.rotationsToDegrees(getGoal().wrist().position));
    goal.addBoolean("At Goal?", this::atGoal);

    ShuffleboardLayout voltages = Telemetry.addColumn(tab, "Voltages");

    voltages.addDouble("Shoulder Voltage (V)", () -> shoulderMotorValues.appliedVolts);
    voltages.addDouble("Elbow Voltage (V)", () -> elbowMotorValues.appliedVolts);

    ShuffleboardLayout velocities = Telemetry.addColumn(tab, "Velocities");

    velocities.addDouble(
        "Shoulder Velocity (degps)",
        () -> Units.rotationsToDegrees(shoulderMotorValues.velocityRotationsPerSecond));
    velocities.addDouble(
        "Elbow Velocity (degps)",
        () -> Units.rotationsToDegrees(elbowMotorValues.velocityRotationsPerSecond));
  }

  public void setPosition(ArmState state) {
    shoulderMotor.setPosition(state.shoulder().position);
    elbowMotor.setPosition(state.elbow().position);
    // wristMotor.setPosition(state.wrist().getRotations());
  }

  /**
   * Gets the position of the arm.
   *
   * @return the position of the arm.
   */
  public ArmState getPosition() {
    shoulderMotor.update(shoulderMotorValues);
    elbowMotor.update(elbowMotorValues);

    return new ArmState(
        Rotation2d.fromRotations(shoulderMotorValues.positionRotations),
        Rotation2d.fromRotations(elbowMotorValues.positionRotations),
        Rotation2d.fromRotations(0));
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
    elbowMotor.setSetpoint(setpoint.elbow().position, setpoint.elbow().velocity);
    // wristMotor.runSetpoint(state.wrist().getRotations());
  }

  public Command to(ArmState goal) {
    return runOnce(() -> setGoal(goal)).andThen(Commands.waitUntil(this::atGoal));
  }

  // TODO Replace with a feedforward call that returns a result
  public TwoJointedArmFeedforward getFeedforward() {
    return feedforward;
  }
}
