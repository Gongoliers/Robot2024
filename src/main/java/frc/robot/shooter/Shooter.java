package frc.robot.shooter;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.lib.Subsystem;
import frc.lib.config.MechanismConfig;
import frc.lib.config.MechanismConfig.MechanismConfigBuilder;
import frc.lib.controller.VelocityControllerIO;
import frc.lib.controller.VelocityControllerIO.VelocityControllerIOValues;

/** Shooter subsystem. */
public class Shooter extends Subsystem {

  /** Shooter subsystem singleton. */
  private static Shooter instance = null;

  /** Flywheel controller config. */
  private final MechanismConfig flywheelConfig =
      MechanismConfigBuilder.defaults()
          .motorConfig(
              motor ->
                  motor
                      .ccwPositive(false)
                      .motorToMechanismRatio(28.0 / 16.0)
                      .statorCurrentLimit(40.0)) // TODO Test, 40A -> 80A?
          .motionProfileConfig(
              motionProfile -> motionProfile.maximumVelocity(50).maximumAcceleration(200))
          .feedforwardControllerConfig(feedforward -> feedforward.kS(0.14).kV(0.2))
          .feedbackControllerConfig(feedback -> feedback.kP(0.4))
          .build();

  /** Flywheel controller. */
  private final VelocityControllerIO flywheel;

  /** Flywheel controller values. */
  private final VelocityControllerIOValues flywheelValues;

  /** Flywheel controller acceleration limiter. */
  private final SlewRateLimiter flywheelAccelerationLimiter;

  /** Serializer controller config. */
  private final MechanismConfig serializerConfig =
      MechanismConfigBuilder.defaults()
          .motorConfig(
              motorConfig ->
                  motorConfig
                      .ccwPositive(true)
                      .motorToMechanismRatio(36.0 / 16.0)
                      .statorCurrentLimit(40.0))
          .motionProfileConfig(
              motionProfileConfig ->
                  motionProfileConfig.maximumVelocity(45).maximumAcceleration(450))
          .feedforwardControllerConfig(feedforwardConfig -> feedforwardConfig.kS(0.14).kV(0.2617))
          .build();

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

    flywheelAccelerationLimiter = flywheelConfig.motionProfileConfig().createAccelerationLimiter();

    serializer = ShooterFactory.createSerializer(serializerConfig);
    serializer.configure();

    serializerValues = new VelocityControllerIOValues();

    serializerAccelerationLimiter =
        serializerConfig.motionProfileConfig().createAccelerationLimiter();

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
}
