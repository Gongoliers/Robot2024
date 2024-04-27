package frc.robot.shooter;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.Subsystem;
import frc.lib.config.FeedbackControllerConfig;
import frc.lib.config.FeedforwardControllerConfig;
import frc.lib.config.MechanismConfig;
import frc.lib.config.MotionProfileConfig;
import frc.lib.config.MotorConfig;
import frc.lib.controller.VelocityControllerIO;
import frc.lib.controller.VelocityControllerIO.VelocityControllerIOValues;

/** Shooter subsystem. */
public class Shooter extends Subsystem {

  /** Shooter singleton. */
  private static Shooter instance = null;

  /** Flywheel controller config. */
  private final MechanismConfig flywheelConfig =
      new MechanismConfig()
          .withMotorConfig(
              new MotorConfig()
                  .withCCWPositive(false)
                  .withNeutralBrake(true)
                  .withMotorToMechanismRatio(28.0 / 16.0))
          .withMotionProfileConfig(
              new MotionProfileConfig()
                  .withMaximumVelocity(60) // rotations per second
                  .withMaximumAcceleration(200) // rotations per second per second
              )
          .withFeedforwardConfig(
              new FeedforwardControllerConfig()
                  .withStaticFeedforward(0.14) // volts
                  .withVelocityFeedforward(0.2) // volts per rotation per second
              )
          .withFeedbackConfig(
              new FeedbackControllerConfig()
                  .withProportionalGain(0.14) // volts per rotation per second
              );

  /** Flywheel controller. */
  private final VelocityControllerIO flywheel;

  /** Flywheel controller values. */
  private final VelocityControllerIOValues flywheelValues;

  /** Flywheel controller acceleration limiter. */
  private final SlewRateLimiter flywheelAccelerationLimiter;

  /** Serializer controller config. */
  private final MechanismConfig serializerConfig =
      new MechanismConfig()
          .withMotorConfig(
              new MotorConfig()
                  .withCCWPositive(true)
                  .withNeutralBrake(false)
                  .withMotorToMechanismRatio(36.0 / 16.0))
          .withMotionProfileConfig(
              new MotionProfileConfig()
                  .withMaximumVelocity(45) // rotations per second
                  .withMaximumAcceleration(450) // rotations per second per second
              )
          .withFeedforwardConfig(
              new FeedforwardControllerConfig()
                  .withStaticFeedforward(0.14) // volts
                  .withVelocityFeedforward(0.2617) // volts per rotation per second
              );

  /** Serializer controller. */
  private final VelocityControllerIO serializer;

  /** Serializer controller values. */
  private final VelocityControllerIOValues serializerValues;

  /** Serializer controller acceleration limiter. */
  private final SlewRateLimiter serializerAccelerationLimiter;

  /** Shooter goal. Set by superstructure. */
  private ShooterState goal;

  /** Shooter setpoint. Updated periodically to reach goal within constraints. */
  private ShooterState setpoint;

  /** Initializes the shooter subsystem and configures shooter hardware. */
  private Shooter() {
    flywheel = ShooterFactory.createFlywheel(flywheelConfig);
    flywheel.configure();

    flywheelValues = new VelocityControllerIOValues();

    flywheelAccelerationLimiter = flywheelConfig.motionProfileConfig().createRateLimiter();

    serializer = ShooterFactory.createSerializer(serializerConfig);
    serializer.configure();

    serializerValues = new VelocityControllerIOValues();

    serializerAccelerationLimiter = serializerConfig.motionProfileConfig().createRateLimiter();

    setpoint = ShooterState.IDLING;
    goal = ShooterState.IDLING;
  }

  /**
   * Returns the shooter subsystem instance.
   *
   * @return the shooter subsystem instance.
   */
  public static Shooter getInstance() {
    if (instance == null) {
      instance = new Shooter();
    }

    return instance;
  }

  @Override
  public void periodic() {
    flywheel.update(flywheelValues);
    serializer.update(serializerValues);

    setpoint = goal;

    double flywheelSetpoint =
        flywheelAccelerationLimiter.calculate(setpoint.flywheelVelocityRotationsPerSecond());
    double serializerSetpoint =
        serializerAccelerationLimiter.calculate(setpoint.serializerVelocityRotationsPerSecond());

    flywheel.setSetpoint(flywheelSetpoint);
    serializer.setSetpoint(serializerSetpoint);
  }

  @Override
  public void addToShuffleboard(ShuffleboardTab tab) {
    VelocityControllerIO.addToShuffleboard(tab, "Flywheel", flywheelValues);
    VelocityControllerIO.addToShuffleboard(tab, "Serializer", serializerValues);

    tab.addBoolean("Serializer Current Spike?", this::serializerCurrentSpike);
  }

  /**
   * Returns the shooter state.
   *
   * @return the shooter state.
   */
  public ShooterState getState() {
    return new ShooterState(
        flywheelValues.velocityRotationsPerSecond, serializerValues.velocityRotationsPerSecond);
  }

  /**
   * Sets the shooter goal state.
   *
   * @param goal the shooter goal state.
   */
  public void setGoal(ShooterState goal) {
    this.goal = goal;
  }

  /**
   * Returns true if the shooter is at the goal state.
   *
   * @return true if the shooter is at the goal state.
   */
  public boolean atGoal() {
    return getState().at(goal);
  }

  /**
   * Returns a trigger for if a note is serialized.
   *
   * @return a trigger for if a note is serialized.
   */
  public Trigger serializedNote() {
    return new Trigger(this::serializerCurrentSpike);
  }

  /**
   * Returns true if the serializer has a current spike.
   *
   * @return true if the serializer has a current spike.
   */
  private boolean serializerCurrentSpike() {
    return serializerValues.motorAmps > 20.0;
  }
}
