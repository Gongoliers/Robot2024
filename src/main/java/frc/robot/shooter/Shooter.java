package frc.robot.shooter;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.Subsystem;
import frc.lib.controller.VelocityControllerIO;
import frc.lib.controller.VelocityControllerIO.VelocityControllerIOValues;
import frc.robot.shooter.ShooterConstants.FlywheelConstants;
import frc.robot.shooter.ShooterConstants.SerializerConstants;

/** Subsystem class for the shooter subsystem. */
public class Shooter extends Subsystem {

  /** Instance variable for the shooter subsystem singleton. */
  private static Shooter instance = null;

  /** Serializer. */
  private final VelocityControllerIO serializer;

  /** Serializer values. */
  private final VelocityControllerIOValues serializerValues;

  /** Flywheel. */
  private final VelocityControllerIO flywheel;

  /** Flywheel values. */
  private final VelocityControllerIOValues flywheelValues;

  private ShooterState setpoint, goal;

  /** Creates a new instance of the shooter subsystem. */
  private Shooter() {
    serializer = ShooterFactory.createSerializer();
    serializerValues = new VelocityControllerIOValues();
    serializer.configure(SerializerConstants.CONTROLLER_CONSTANTS);

    flywheel = ShooterFactory.createFlywheel();
    flywheelValues = new VelocityControllerIOValues();
    flywheel.configure(FlywheelConstants.CONTROLLER_CONSTANTS);

    setpoint = ShooterState.IDLE;
    goal = ShooterState.IDLE;
  }

  /**
   * Gets the instance of the shooter subsystem.
   *
   * @return the instance of the shooter subsystem.
   */
  public static Shooter getInstance() {
    if (instance == null) {
      instance = new Shooter();
    }

    return instance;
  }

  @Override
  public void periodic() {
    serializer.update(serializerValues);
    flywheel.update(flywheelValues);

    setpoint = goal;

    double flywheelSetpoint = FlywheelConstants.ACCELERATION_LIMITER.calculate(setpoint.flywheelVelocityRotationsPerSecond());
    double serializerSetpoint = SerializerConstants.ACCELERATION_LIMITER.calculate(setpoint.serializerVelocityRotationsPerSecond());

    flywheel.setSetpoint(flywheelSetpoint);
    serializer.setSetpoint(serializerSetpoint);
  }

  @Override
  public void addToShuffleboard(ShuffleboardTab tab) {
    VelocityControllerIO.addToShuffleboard(tab, "Serializer", serializerValues);
    VelocityControllerIO.addToShuffleboard(tab, "Flywheel", flywheelValues);
  }

  public Trigger serializedNote() {
    return new Trigger(() -> serializerValues.motorAmps > SerializerConstants.NOTE_AMPS);
  } 

  public ShooterState getState() {
    return new ShooterState(
        flywheelValues.velocityRotationsPerSecond, serializerValues.velocityRotationsPerSecond);
  }

  public void setGoal(ShooterState goal) {
    this.goal = goal;
  }

  public boolean atGoal() {
    return getState().at(goal);
  }
}
