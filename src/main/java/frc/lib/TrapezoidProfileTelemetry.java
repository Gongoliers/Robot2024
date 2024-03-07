package frc.lib;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.DoubleEntry;

public class TrapezoidProfileTelemetry {

  private final DoubleEntry measuredPosition, measuredVelocity;
  private final DoubleEntry setpointPosition, setpointVelocity;
  private final DoubleEntry goalPosition, goalVelocity;

  public TrapezoidProfileTelemetry(String table) {
    measuredPosition = Telemetry.addDoubleEntry(table, "measuredPosition");
    measuredVelocity = Telemetry.addDoubleEntry(table, "measuredVelocity");
    setpointPosition = Telemetry.addDoubleEntry(table, "setpointPosition");
    setpointVelocity = Telemetry.addDoubleEntry(table, "setpointVelocity");
    goalPosition = Telemetry.addDoubleEntry(table, "goalPosition");
    goalVelocity = Telemetry.addDoubleEntry(table, "goalVelocity");
  }

  public void updateMeasurement(double position, double velocity) {
    measuredPosition.set(position);
    measuredVelocity.set(velocity);
  }

  public void updateMeasurement(State state) {
    updateMeasurement(state.position, state.velocity);
  }

  public void updateSetpoint(double position, double velocity) {
    setpointPosition.set(position);
    setpointVelocity.set(velocity);
  }

  public void updateSetpoint(State state) {
    updateSetpoint(state.position, state.velocity);
  }

  public void updateGoal(double position, double velocity) {
    goalPosition.set(position);
    goalVelocity.set(velocity);
  }

  public void updateGoal(State state) {
    updateGoal(state.position, state.velocity);
  }

  public void update(State measurement, State setpoint, State goal) {
    updateMeasurement(measurement);
    updateSetpoint(setpoint);
    updateGoal(goal);
  }
}
