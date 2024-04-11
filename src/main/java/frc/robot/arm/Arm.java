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

/** Subsystem class for the arm subsystem. */
public class Arm extends Subsystem {

  /** Instance variable for the arm subsystem singleton. */
  private static Arm instance = null;

  /** Shoulder. */
  private final PositionControllerIO shoulder;

  /** Shoulder values. */
  private final PositionControllerIOValues shoulderValues;

  private double lastTimeSeconds;

  /** Arm goal. */
  private ArmState setpoint, goal;

  /** Creates a new instance of the arm subsystem. */
  private Arm() {
    shoulder = ArmFactory.createShoulder();
    shoulderValues = new PositionControllerIOValues();
    shoulder.configure(ShoulderConstants.CONTROLLER_CONSTANTS);

    lastTimeSeconds = Timer.getFPGATimestamp();

    setPosition(ArmState.INITIAL);
    setpoint = ArmState.INITIAL;
    goal = ArmState.INITIAL;
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

    double dt = timeSeconds - lastTimeSeconds;

    lastTimeSeconds = timeSeconds;

    setpoint =
        new ArmState(
            ShoulderConstants.MOTION_PROFILE.calculate(
                dt,
                setpoint.shoulderRotations(),
                goal.shoulderRotations()));

    shoulder.setSetpoint(
        setpoint.shoulderRotations().position, setpoint.shoulderRotations().velocity);
  }

  @Override
  public void addToShuffleboard(ShuffleboardTab tab) {
    PositionControllerIO.addToShuffleboard(tab, "Shoulder", shoulderValues);

    Telemetry.addColumn(tab, "Setpoint").addDouble("Setpoint (deg)", () -> Units.rotationsToDegrees(setpoint.shoulderRotations().position));
  }

  public ArmState getState() {
    return new ArmState(
        new TrapezoidProfile.State(
            shoulderValues.positionRotations, shoulderValues.velocityRotationsPerSecond));
  }

  public void setGoal(ArmState goal) {
    this.goal = goal;
  }

  public boolean atGoal() {
    return getState().at(goal);
  }

  public void setPosition(ArmState armState) {
    shoulder.setPosition(armState.shoulderRotations().position);
  }
}
