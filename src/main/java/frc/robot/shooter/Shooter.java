package frc.robot.shooter;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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

    setpoint = new ShooterState(0, 0);
    goal = new ShooterState(0, 0);
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

    flywheel.setSetpoint(setpoint.flywheelVelocityRotationsPerSecond());
    serializer.setSetpoint(setpoint.serializerVelocityRotationsPerSecond());
  }

  @Override
  public void addToShuffleboard(ShuffleboardTab tab) {
    VelocityControllerIO.addToShuffleboard(tab, "Serializer", serializerValues);
    VelocityControllerIO.addToShuffleboard(tab, "Flywheel", flywheelValues);
  }

  public ShooterState getState() {
    return new ShooterState(flywheelValues.velocityRotationsPerSecond, serializerValues.velocityRotationsPerSecond);
  }

  public void setGoal(ShooterState goal)  {
    this.goal = goal;
  }

  public boolean atGoal() {
    return getState().at(goal);
  }

}
