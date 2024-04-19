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

  /** Instance variable for the arm subsystem singleton. */
  private static Arm instance = null;

  /** Shoulder position controller. */
  private final PositionControllerIO shoulder;

  /** Shoulder position controller values. */
  private final PositionControllerIOValues shoulderValues;

  /** Arm's goal. Set by superstructure to use as end goal for setpoints. */
  private ArmState goal;

  /** Arm's setpoint. Updated periodically to reach the goal within constraints. */
  private ArmState setpoint;

  /** Time of the start of the previous periodic call. Used for calculating the time elapsed since the previous setpoint was generated. */
  private double previousTimeSeconds;

  /** Creates a new arm subsystem and configures arm hardware. */
  private Arm() {
    shoulder = ArmFactory.createShoulder();
    shoulder.configure(ShoulderConstants.CONTROLLER_CONSTANTS);

    shoulderValues = new PositionControllerIOValues();

    previousTimeSeconds = Timer.getFPGATimestamp();

    setPosition(ArmState.STOW_POSITION);
    setpoint = ArmState.STOW_POSITION;
    goal = ArmState.STOW_POSITION;
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

    // Calculate the time elapsed in seconds since the previous setpoint was generated
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
   * Returns true if at the arm's set goal.
   * 
   * @return true if at the arm's set goal.
   */
  public boolean atGoal() {
    return getState().at(goal);
  }

  /**
   * Sets the position of the shoulder motor encoders.
   * 
   * @param armState the position.
   */
  private void setPosition(ArmState armState) {
    shoulder.setPosition(armState.shoulderRotations().position);
  }
}
