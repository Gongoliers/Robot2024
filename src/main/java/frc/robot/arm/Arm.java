package frc.robot.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.lib.Subsystem;
import frc.lib.controller.PositionControllerIO;
import frc.lib.controller.PositionControllerIO.PositionControllerIOValues;
import frc.robot.RobotConstants;
import frc.robot.arm.ArmConstants.ShoulderConstants;
import frc.robot.superstructure.SuperstructureConstants;
import frc.robot.superstructure.SuperstructureConstants.ShoulderAngleConstants;

/** Subsystem class for the arm subsystem. */
public class Arm extends Subsystem {

  /** Instance variable for the arm subsystem singleton. */
  private static Arm instance = null;

  /** Shoulder. */
  private final PositionControllerIO shoulder;

  /** Shoulder values. */
  private final PositionControllerIOValues shoulderValues;

  /** Arm goal. */
  private State setpoint, goal;

  /** Creates a new instance of the arm subsystem. */
  private Arm() {
    shoulder = ArmFactory.createShoulder();
    shoulderValues = new PositionControllerIOValues();
    shoulder.configure(ShoulderConstants.CONTROLLER_CONSTANTS);

    setPosition(ShoulderAngleConstants.INITIAL.getRotations());
    setpoint = new State(ShoulderAngleConstants.INITIAL.getRotations(), 0);
    goal = new State(ShoulderAngleConstants.INITIAL.getRotations(), 0);
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

    setpoint = ShoulderAngleConstants.MOTION_PROFILE.calculate(
        RobotConstants.PERIODIC_DURATION,
        setpoint,
        goal);

    shoulder.setSetpoint(setpoint.position, setpoint.velocity);
  }

  @Override
  public void addToShuffleboard(ShuffleboardTab tab) {
    PositionControllerIO.addToShuffleboard(tab, "Shoulder", shoulderValues);
  }

  public State getState() {
    return new TrapezoidProfile.State(
        shoulderValues.positionRotations, shoulderValues.velocityRotationsPerSecond);
  }

  public void setGoal(State goal) {
    this.goal = goal;
  }

  public boolean atGoal() {
    return MathUtil.isNear(
        getState().position,
        goal.position,
        SuperstructureConstants.ShoulderAngleConstants.TOLERANCE.getRotations());
  }

  public void setPosition(double positionRotations) {
    shoulder.setPosition(positionRotations);
  }
}
