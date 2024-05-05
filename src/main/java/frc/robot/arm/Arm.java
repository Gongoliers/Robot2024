package frc.robot.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.lib.Subsystem;
import frc.lib.Telemetry;
import frc.lib.config.MechanismConfig;
import frc.lib.config.MechanismConfig.MechanismConfigBuilder;
import frc.lib.controller.PositionControllerIO;
import frc.lib.controller.PositionControllerIO.PositionControllerIOValues;

/** Arm subsystem. */
public class Arm extends Subsystem {

  /** Arm subsystem singleton. */
  private static Arm instance = null;

  /** Shoulder controller config. */
  private final MechanismConfig shoulderConfig =
      MechanismConfigBuilder.defaults()
          .absoluteEncoderConfig(
              absoluteEncoder ->
                  absoluteEncoder.ccwPositive(false).offset(Rotation2d.fromDegrees(-173.135)))
          .motorConfig(
              motor ->
                  motor
                      .ccwPositive(true)
                      .neutralBrake(true)
                      .motorToMechanismRatio(39.771428571)
                      .statorCurrentLimit(40.0))
          .motionProfileConfig(
              motionProfile ->
                  motionProfile
                      .maximumVelocity(Units.degreesToRotations(240.0))
                      .maximumAcceleration(Units.degreesToRotations(240.0)))
          .feedforwardControllerConfig(feedforward -> feedforward.kS(0.14).kG(0.5125).kV(4.0))
          .feedbackControllerConfig(feedback -> feedback.kP(4.0))
          .build();

  /** Shoulder controller. */
  private final PositionControllerIO shoulder;

  /** Shoulder controller values. */
  private final PositionControllerIOValues shoulderValues;

  /** Motion profile of the shoulder. */
  private final TrapezoidProfile shoulderMotionProfile;

  /** Arm goal. Set by superstructure. */
  private ArmState goal;

  /** Arm setpoint. Updated periodically to reach goal within constraints. */
  private ArmState setpoint;

  /**
   * Time of the previous periodic call. Used for calculating the time elapsed between generating
   * setpoints.
   */
  private double previousTimeSeconds;

  /** Initializes the arm subsystem and configures arm hardware. */
  private Arm() {
    shoulder = ArmFactory.createShoulder(shoulderConfig);
    shoulder.configure();

    shoulderValues = new PositionControllerIOValues();

    shoulderMotionProfile = shoulderConfig.motionProfileConfig().createTrapezoidProfile();

    previousTimeSeconds = Timer.getFPGATimestamp();

    setpoint = ArmState.STOWED;
    goal = ArmState.STOWED;
  }

  /**
   * Returns the arm subsystem instance.
   *
   * @return the arm subsystem instance.
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
            shoulderMotionProfile.calculate(
                timeElapsedSeconds, setpoint.shoulderRotations(), goal.shoulderRotations()));

    shoulder.setSetpoint(
        setpoint.shoulderRotations().position, setpoint.shoulderRotations().velocity);

    previousTimeSeconds = timeSeconds;
  }

  @Override
  public void addToShuffleboard(ShuffleboardTab tab) {
    PositionControllerIO.addToShuffleboard(tab, "Shoulder", shoulderValues);

    Telemetry.addColumn(tab, "Setpoint")
        .addDouble("Setpoint (rot)", () -> setpoint.shoulderRotations().position);
  }

  /**
   * Returns the arm state.
   *
   * @return the arm state.
   */
  public ArmState getState() {
    return new ArmState(
        new TrapezoidProfile.State(
            shoulderValues.positionRotations, shoulderValues.velocityRotationsPerSecond));
  }

  /**
   * Sets the arm goal state.
   *
   * @param goal the arm goal state.
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
