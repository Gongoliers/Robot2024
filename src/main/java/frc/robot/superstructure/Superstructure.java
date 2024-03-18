package frc.robot.superstructure;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.lib.Subsystem;
import frc.lib.Telemetry;
import frc.robot.RobotMechanisms;
import frc.robot.arm.Arm;
import frc.robot.intake.Intake;
import frc.robot.shooter.Shooter;

/** Subsystem class for the superstructure subsystem. */
public class Superstructure extends Subsystem {

  /** Instance variable for the superstructure subsystem singleton. */
  private static Superstructure instance = null;

  /** Reference to the arm subsystem. */
  private final Arm arm;

  /** Reference to the intake subsystem. */
  private final Intake intake;

  /** Reference to the shooter subsystem. */
  private final Shooter shooter;

  private SuperstructureState measurement;

  /** Creates a new instance of the superstructure subsystem. */
  private Superstructure() {
    arm = Arm.getInstance();
    intake = Intake.getInstance();
    shooter = Shooter.getInstance();
  }

  /**
   * Gets the instance of the superstructure subsystem.
   *
   * @return the instance of the superstructre subsystem.
   */
  public static Superstructure getInstance() {
    if (instance == null) {
      instance = new Superstructure();
    }

    return instance;
  }

  @Override
  public void periodic() {
    State measuredShoulderState = arm.getMeasuredShoulderState();
    State measuredWristState = arm.getMeasuredWristState();

    State measuredIntakePivotState = intake.getMeasuredPivotState();
    double measuredIntakeRollerVelocity = intake.getRollerVelocity();

    double measuredShooterFlywheelVelocity = shooter.getFlywheelVelocity();
    double measuredShooterSerializerVelocity = shooter.getSerializerVelocity();

    measurement =
        new SuperstructureState(
            measuredShoulderState,
            measuredWristState,
            measuredIntakePivotState,
            measuredIntakeRollerVelocity,
            measuredShooterFlywheelVelocity,
            measuredShooterSerializerVelocity);

    RobotMechanisms.getInstance().updateSuperstructure(measurement);
  }

  @Override
  public void addToShuffleboard(ShuffleboardTab tab) {
    ShuffleboardLayout measurement = Telemetry.addColumn(tab, "Measurement");

    measurement.addDouble(
        "Shoulder Position (deg)",
        () -> Units.rotationsToDegrees(this.measurement.shoulderAngleRotations().position));
    measurement.addDouble(
        "Shoulder Velocity (dps)",
        () -> Units.rotationsToDegrees(this.measurement.shoulderAngleRotations().velocity));

    measurement.addDouble(
        "Wrist Position (deg)",
        () -> Units.rotationsToDegrees(this.measurement.wristAngleRotations().position));
    measurement.addDouble(
        "Wrist Velocity (dps)",
        () -> Units.rotationsToDegrees(this.measurement.wristAngleRotations().velocity));

    measurement.addDouble(
        "Pivot Position (deg)",
        () -> Units.rotationsToDegrees(this.measurement.pivotAngleRotations().position));
    measurement.addDouble(
        "Pivot Velocity (dps)",
        () -> Units.rotationsToDegrees(this.measurement.pivotAngleRotations().velocity));

    measurement.addDouble(
        "Roller Velocity (rps)", () -> this.measurement.rollerVelocityRotationsPerSecond());

    measurement.addDouble(
        "Flywheel Velocity (rps)", () -> this.measurement.flywheelVelocityRotationsPerSecond());

    measurement.addDouble(
        "Serializer Velocity (rps)", () -> this.measurement.serializerVelocityRotationsPerSecond());
  }
}
