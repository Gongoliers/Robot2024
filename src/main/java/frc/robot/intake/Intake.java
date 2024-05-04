package frc.robot.intake;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.Subsystem;
import frc.lib.config.MechanismConfig;
import frc.lib.config.MechanismConfig.MechanismConfigBuilder;
import frc.lib.controller.VelocityControllerIO;
import frc.lib.controller.VelocityControllerIO.VelocityControllerIOValues;

/** Intake subsystem. */
public class Intake extends Subsystem {

  /** Intake subsystem singleton. */
  private static Intake instance = null;

  /** Front roller controller config. */
  private final MechanismConfig frontRollerConfig =
      MechanismConfigBuilder.defaults()
          .motorConfig(
              motor ->
                  motor.ccwPositive(false).neutralBrake(false).motorToMechanismRatio(24.0 / 16.0))
          .motionProfileConfig(motionProfile -> motionProfile.maximumVelocity(66))
          .feedforwardControllerConfig(feedforward -> feedforward.kS(0.13).kV(0.1683))
          .feedbackControllerConfig(feedback -> feedback.kP(0.1))
          .build();

  /** Back roller controller config. */
  private final MechanismConfig backRollerConfig =
      MechanismConfigBuilder.defaults()
          .motorConfig(
              motor ->
                  motor.ccwPositive(false).neutralBrake(false).motorToMechanismRatio(24.0 / 16.0))
          .motionProfileConfig(motionProfile -> motionProfile.maximumVelocity(66))
          .feedforwardControllerConfig(feedforward -> feedforward.kS(0.13).kV(0.1759))
          .feedbackControllerConfig(feedback -> feedback.kP(0.1))
          .build();

  /** Rollers. */
  private final VelocityControllerIO frontRoller, backRoller;

  /** Roller values. */
  private final VelocityControllerIOValues frontRollerValues, backRollerValues;

  /** Intake goal. Set by superstructure. */
  private IntakeState goal;

  /** Intake setpoint. Updated periodically to reach goal within constraints. */
  private IntakeState setpoint;

  /** Initializes the intake subsystem and configures intake hardware. */
  private Intake() {
    frontRoller = IntakeFactory.createFrontRoller(frontRollerConfig);
    frontRoller.configure();

    frontRollerValues = new VelocityControllerIOValues();

    backRoller = IntakeFactory.createBackRoller(backRollerConfig);
    backRoller.configure();

    backRollerValues = new VelocityControllerIOValues();

    setpoint = IntakeState.IDLE;
    goal = IntakeState.IDLE;
  }

  /**
   * Returns the intake subsystem instance.
   *
   * @return the intake subsystem instance.
   */
  public static Intake getInstance() {
    if (instance == null) {
      instance = new Intake();
    }

    return instance;
  }

  @Override
  public void periodic() {
    frontRoller.update(frontRollerValues);
    backRoller.update(backRollerValues);

    setpoint = goal;

    frontRoller.setSetpoint(setpoint.frontRollerVelocityRotationsPerSecond());
    backRoller.setSetpoint(setpoint.backRollerVelocityRotationsPerSecond());
  }

  @Override
  public void addToShuffleboard(ShuffleboardTab tab) {
    VelocityControllerIO.addToShuffleboard(tab, "Front Roller", frontRollerValues);
    VelocityControllerIO.addToShuffleboard(tab, "Back Roller", backRollerValues);
  }

  /**
   * Returns the intake state.
   *
   * @return the intake state.
   */
  public IntakeState getState() {
    return new IntakeState(
        frontRollerValues.velocityRotationsPerSecond, backRollerValues.velocityRotationsPerSecond);
  }

  /**
   * Sets the intake goal state.
   *
   * @param goal the intake goal state.
   */
  public void setGoal(IntakeState goal) {
    this.goal = goal;
  }

  /**
   * Returns true if the intake is at the goal state.
   *
   * @return true if the intake is at the goal state.
   */
  public boolean atGoal() {
    return getState().at(goal);
  }

  /**
   * Returns a trigger for if a note is stuck.
   *
   * @return a trigger for if a note is stuck.
   */
  public Trigger noteStuck() {
    return new Trigger(() -> frontRollerStuck() || backRollerStuck());
  }

  /**
   * Returns true if the front roller is stuck.
   *
   * @return true if the front roller is stuck.
   */
  private boolean frontRollerStuck() {
    return frontRollerValues.motorAmps > 40.0;
  }

  /**
   * Returns true if the back roller is stuck.
   *
   * @return true if the back roller is stuck.
   */
  private boolean backRollerStuck() {
    return backRollerValues.motorAmps > 40.0;
  }
}
