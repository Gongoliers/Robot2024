package frc.robot.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.Subsystem;
import frc.lib.Telemetry;
import frc.robot.arm.ArmConstants.ElbowMotorConstants;
import frc.robot.arm.ArmConstants.ShoulderMotorConstants;
import frc.robot.arm.ElbowMotorIO.ElbowMotorIOValues;
import frc.robot.arm.ShoulderMotorIO.ShoulderMotorIOValues;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

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

  /** Creates a new instance of the arm subsystem. */
  private Arm() {
    shoulderMotor = ArmFactory.createShoulderMotor();
    elbowMotor = ArmFactory.createElbowMotor();
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
    ShuffleboardLayout shoulder = Telemetry.addColumn(tab, "Shoulder");

    shoulder.addDouble("Position (rot)", () -> shoulderMotorValues.positionRotations);

    ShuffleboardLayout elbow = Telemetry.addColumn(tab, "Elbow");

    elbow.addDouble("Position (rot)", () -> elbowMotorValues.positionRotations);
  }

  /**
   * Gets the state of the arm.
   *
   * @return the state of the arm.
   */
  public ArmState getState() {
    return new ArmState(
        Rotation2d.fromRotations(shoulderMotorValues.positionRotations),
        Rotation2d.fromRotations(elbowMotorValues.positionRotations),
        Rotation2d.fromRotations(0));
  }

  private void runSetpoint(ArmState state) {
    shoulderMotor.runSetpoint(state.shoulder().getRotations());
    elbowMotor.runSetpoint(state.elbow().getRotations());
    // wristMotor.runSetpoint(state.wrist().getRotations());
  }

  public Command runSetpoint(Supplier<ArmState> stateSupplier) {
    return run(() -> runSetpoint(stateSupplier.get()));
  }

  public Command hold() {
    return runSetpoint(this::getState);
  }

  public Command driveShoulderWithJoystick(DoubleSupplier joystickSupplier) {
    return run(() ->
            shoulderMotor.setVoltage(
                joystickSupplier.getAsDouble() * ShoulderMotorConstants.MAXIMUM_VOLTAGE))
        .finallyDo(shoulderMotor::stop);
  }

  public Command driveElbowWithJoystick(DoubleSupplier joystickSupplier) {
    return run(() ->
            elbowMotor.setVoltage(
                joystickSupplier.getAsDouble() * ElbowMotorConstants.MAXIMUM_VOLTAGE))
        .finallyDo(shoulderMotor::stop);
  }
}
