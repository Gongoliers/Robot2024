package frc.robot.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.Subsystem;
import frc.lib.Telemetry;
import frc.robot.arm.ArmConstants.ElbowMotorConstants;
import frc.robot.arm.ElbowMotorIO.ElbowMotorIOValues;
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

  /** Elbow motor. */
  private final ElbowMotorIO elbowMotor;

  /** Elbow motor values. */
  private final ElbowMotorIOValues elbowMotorValues = new ElbowMotorIOValues();

  private ArmState goal, setpoint;

  /** Creates a new instance of the arm subsystem. */
  private Arm() {
    shoulderMotor = ArmFactory.createShoulderMotor();
    elbowMotor = ArmFactory.createElbowMotor();

    setPosition(ArmState.UP);

    goal = getPosition();
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

    ArmMechanism.getInstance().setState(getPosition());
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

    ShuffleboardLayout goal = Telemetry.addColumn(tab, "Goal");

    goal.addDouble(
        "Shoulder Setpoint (deg)", () -> Units.rotationsToDegrees(getGoal().shoulder().position));
    goal.addDouble(
        "Elbow Setpoint (deg)", () -> Units.rotationsToDegrees(getGoal().elbow().position));
    goal.addDouble(
        "Wrist Setpoint (deg)", () -> Units.rotationsToDegrees(getGoal().wrist().position));
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

  private void setGoal(ArmState goal) {
    this.goal = goal;
    setSetpoint(getSetpoint().nextSetpoint(goal));
  }

  public ArmState getSetpoint() {
    return setpoint;
  }

  private void setSetpoint(ArmState setpoint) {
    this.setpoint = setpoint;

    shoulderMotor.setSetpoint(setpoint.shoulder().position, setpoint.shoulder().velocity);
    elbowMotor.setSetpoint(setpoint.elbow().position);
    // wristMotor.runSetpoint(state.wrist().getRotations());
  }

  public Command runGoal(ArmState goal) {
    return run(() -> setGoal(goal));
  }

  public Command hold() {
    return run(() -> setSetpoint(getGoal())).beforeStarting(() -> setGoal(getPosition()));
  }

  public Command driveElbowWithJoystick(DoubleSupplier joystickSupplier) {
    return run(() ->
            elbowMotor.setVoltage(
                joystickSupplier.getAsDouble()
                    * -1
                    * Math.abs(ElbowMotorConstants.MAXIMUM_VOLTAGE)))
        .finallyDo(elbowMotor::stop);
  }
}
