package frc.robot.arm;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.lib.Subsystem;
import frc.lib.Telemetry;
import frc.lib.controller.PositionControllerIO;
import frc.lib.controller.PositionControllerIO.PositionControllerIOValues;
import frc.robot.arm.ArmConstants.ShoulderConstants;

/** Arm subsystem. */
public class Arm extends Subsystem {

  /** Arm subsystem singleton. */
  private static Arm instance = null;

  /** Shoulder controller. */
  private final PositionControllerIO shoulder;

  /** Shoulder controller values. */
  private final PositionControllerIOValues shoulderValues;

  /** Arm's goal. Set by superstructure. */
  private ArmState goal;

  /** Arm's setpoint. Updated periodically to reach goal within constraints. */
  private ArmState setpoint;

  /**
   * Time of the previous periodic call. Used for calculating the time elapsed between generating
   * setpoints.
   */
  private double previousTimeSeconds;

  /** Creates the arm subsystem and configures arm hardware. */
  private Arm() {
    shoulder = ArmFactory.createShoulder();
    shoulder.configure();

    shoulderValues = new PositionControllerIOValues();

    previousTimeSeconds = Timer.getFPGATimestamp();

    setPosition(ArmState.STOW);
    setpoint = ArmState.STOW;
    goal = ArmState.STOW;
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

    double timeSeconds = Timer.getFPGATimestamp();

    // Calculate the time elapsed since the previous setpoint was generated
    double timeElapsedSeconds = timeSeconds - previousTimeSeconds;

    setpoint =
        new ArmState(
            ShoulderConstants.MOTION_PROFILE.calculate(
                timeElapsedSeconds, setpoint.shoulderRotations(), goal.shoulderRotations()));

    shoulder.setSetpoint(
        setpoint.shoulderRotations().position, setpoint.shoulderRotations().velocity);

    previousTimeSeconds = timeSeconds;
  }

  @Override
  public void addToShuffleboard(ShuffleboardTab tab) {
    PositionControllerIO.addToShuffleboard(tab, "Shoulder", shoulderValues);

    Telemetry.addColumn(tab, "Setpoint")
        .addDouble(
            "Setpoint (deg)",
            () -> Units.rotationsToDegrees(setpoint.shoulderRotations().position));
  }

  /**
   * Returns the arm's state.
   *
   * @return the arm's state.
   */
  public ArmState getState() {
    return new ArmState(
        new TrapezoidProfile.State(
            shoulderValues.positionRotations, shoulderValues.velocityRotationsPerSecond));
  }

  /**
   * Sets the arm's goal state.
   *
   * @param goal the arm's goal state.
   */
  public void setGoal(ArmState goal) {
    this.goal = goal;
  }

  /**
   * Returns true if the arm is at the goal state.
   *
   * @return true if the arm is at the goal state.
   */
  public boolean atGoal() {
    return getState().at(goal);
  }

  /**
   * Sets the position.
   *
   * @param state the state to use as position.
   */
  public void setPosition(ArmState state) {
    shoulder.setPosition(state.shoulderRotations().position);
  }
}
