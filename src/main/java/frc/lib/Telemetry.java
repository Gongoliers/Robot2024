package frc.lib;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import java.util.HashMap;
import java.util.function.Supplier;

/** Helper class for managing robot telemetry. */
public class Telemetry {

  private static final HashMap<String, Integer> tabColumnCount = new HashMap<>();

  /**
   * Initializes a subsystem's Shuffleboard tab.
   *
   * @param subsystem the subsystem to initialize.
   */
  public static void initializeShuffleboard(Subsystem subsystem) {
    String name = subsystem.getName();

    ShuffleboardTab tab = Shuffleboard.getTab(name);

    subsystem.addToShuffleboard(tab);
  }

  /**
   * Initializes each subsystem's Shuffleboard tab.
   *
   * @param subsystems the subsystems to initialize.
   */
  public static void initializeShuffleboards(Subsystem... subsystems) {
    for (Subsystem subsystem : subsystems) {
      Telemetry.initializeShuffleboard(subsystem);
    }
  }

  /**
   * Adds a column to a Shuffleboard tab.
   *
   * @param tab the Shuffleboard tab to add the column to.
   * @param columnTitle the title of the column.
   * @return the added Shuffleboard column.
   */
  public static ShuffleboardLayout addColumn(ShuffleboardTab tab, String columnTitle) {
    String tabTitle = tab.getTitle();

    int currentColumnCount = tabColumnCount.getOrDefault(tabTitle, 0);
    // Increment the column count for the next call
    tabColumnCount.put(tabTitle, currentColumnCount + 1);

    final int kColumnWidth = 2;
    final int kColumnHeight = 4;

    return tab.getLayout(columnTitle, BuiltInLayouts.kList)
        .withSize(kColumnWidth, kColumnHeight)
        .withPosition(currentColumnCount * kColumnWidth, 0);
  }

  /**
   * Adds a fullscreen sendable to a Shuffleboard tab.
   *
   * @param tab the Shuffleboard tab to add the Sendable to.
   * @param title the title of the sendable.
   * @param sendable the sendable to add.
   */
  public static void addFullscreen(ShuffleboardTab tab, String title, Sendable sendable) {
    tab.add(title, sendable).withPosition(0, 0).withSize(10, 4);
  }

  /**
   * Adds swerve module states to a Shuffleboard tab.
   *
   * @param tab the Shuffleboard tab to add the swerve module states to.
   * @param title the title of the swerve module states.
   * @param swerveModuleStatesSupplier a supplier for swerve module states.
   */
  public static void addSwerveModuleStates(
      ShuffleboardTab tab, String title, Supplier<SwerveModuleState[]> swerveModuleStatesSupplier) {
    tab.addDoubleArray(
        title,
        () -> {
          SwerveModuleState[] states = swerveModuleStatesSupplier.get();
          double[] doubles = new double[8];

          for (int i = 0; i < 4; i++) {
            SwerveModuleState state = states[i];
            doubles[2 * i] = state.angle.getDegrees();
            doubles[2 * i + 1] = state.speedMetersPerSecond;
          }

          return doubles;
        });
  }

  /**
   * Adds a boolean entry to the Network Tables table.
   *
   * @param table the Network Tables table.
   * @param name the name of the boolean entry.
   * @param defaultValue the default value of the entry.
   * @return the boolean entry.
   */
  public static BooleanEntry addBooleanEntry(
      NetworkTable table, String name, boolean defaultValue) {
    BooleanEntry entry = table.getBooleanTopic(name).getEntry(defaultValue);

    entry.set(defaultValue);

    return entry;
  }

  /**
   * Adds a boolean entry to the Network Tables table.
   *
   * @param table the Network Tables table.
   * @param name the name of the boolean entry.
   * @return the boolean entry.
   */
  public static BooleanEntry addBooleanEntry(NetworkTable table, String name) {
    return addBooleanEntry(table, name, false);
  }

  /**
   * Adds a double entry to the Network Tables table.
   *
   * @param table the Network Tables table.
   * @param name the name of the boolean entry.
   * @param defaultValue the default value of the entry.
   * @return the double entry.
   */
  public static DoubleEntry addDoubleEntry(NetworkTable table, String name, double defaultValue) {
    DoubleEntry entry = table.getDoubleTopic(name).getEntry(defaultValue);

    entry.set(defaultValue);

    return entry;
  }

  /**
   * Adds a double entry to the Network Tables table.
   *
   * @param table the Network Tables table.
   * @param name the name of the boolean entry.
   * @return the double entry.
   */
  public static DoubleEntry addDoubleEntry(NetworkTable table, String name) {
    return addDoubleEntry(table, name, 0.0);
  }
}
