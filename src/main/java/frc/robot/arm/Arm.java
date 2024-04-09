package frc.robot.arm;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.lib.Subsystem;
import frc.lib.controller.PositionControllerIO;
import frc.lib.controller.PositionControllerIO.PositionControllerIOValues;
import frc.robot.arm.ArmConstants.ShoulderConstants;

/** Subsystem class for the arm subsystem. */
public class Arm extends Subsystem {

  /** Instance variable for the arm subsystem singleton. */
  private static Arm instance = null;

  /** Shoulder. */
  private final PositionControllerIO shoulder;

  /** Shoulder values. */
  private final PositionControllerIOValues shoulderValues;

  /** Creates a new instance of the arm subsystem. */
  private Arm() {
    shoulder = ArmFactory.createShoulder();
    shoulderValues = new PositionControllerIOValues();
    shoulder.configure(ShoulderConstants.CONTROLLER_CONSTANTS);
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
    shoulder.update(shoulderValues);
  }

  @Override
  public void addToShuffleboard(ShuffleboardTab tab) {
    PositionControllerIO.addToShuffleboard(tab, "Shoulder", shoulderValues);
  }

  public State getMeasuredShoulderState() {
    return new TrapezoidProfile.State(
        shoulderValues.positionRotations, shoulderValues.velocityRotationsPerSecond);
  }

  public void setShoulderPosition(double positionRotations) {
    shoulder.setPosition(positionRotations);
  }

  public void setSetpoint(double positionRotations, double velocityRotationsPerSecond) {
    shoulder.setSetpoint(positionRotations, velocityRotationsPerSecond);
  }

  public void setVoltage(double volts) {
    // TODO
  }
}
