package frc.lib;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import java.util.HashMap;
import java.util.function.Supplier;

/** Helper class for managing robot telemetry. */
public class Telemetry {

  private static final HashMap<String, Integer> tabColumn = new HashMap<>();

  /**
   * Initializes a subsystem's Shuffleboard tab.
   *
   * @param subsystem the subsystem to initialize.
   */
  public static void initializeTab(Subsystem subsystem) {
    String name = subsystem.getName();

    ShuffleboardTab tab = Shuffleboard.getTab(name);

    subsystem.addToShuffleboard(tab);
  }

  /**
   * Initializes each subsystem's Shuffleboard tab.
   *
   * @param subsystems the subsystems to initialize.
   */
  public static void initializeTabs(Subsystem... subsystems) {
    for (Subsystem subsystem : subsystems) {
      Telemetry.initializeTab(subsystem);
    }
  }

  /**
   * Adds a column to a Shuffleboard tab.
   *
   * @param tab the Shuffleboard tab to add the column to.
   * @param columnTitle the title of the column.
   * @param width the width of the column.
   * @return the added Shuffleboard column.
   */
  public static ShuffleboardLayout addColumn(ShuffleboardTab tab, String columnTitle, int width) {
    String tabTitle = tab.getTitle();

    int column = tabColumn.containsKey(tabTitle) ? tabColumn.get(tabTitle) + width : 0;

    // Increment the column number for the next call
    tabColumn.put(tabTitle, column);

    final int kColumnHeight = 4;

    return tab.getLayout(columnTitle, BuiltInLayouts.kList)
        .withSize(width, kColumnHeight)
        .withPosition(column, 0);
  }

  /**
   * Adds a column to a Shuffleboard tab.
   *
   * @param tab the Shuffleboard tab to add the column to.
   * @param columnTitle the title of the column.
   * @return the added Shuffleboard column.
   */
  public static ShuffleboardLayout addColumn(ShuffleboardTab tab, String columnTitle) {
    return addColumn(tab, columnTitle, 2);
  }

  /**
   * Makes a Shuffleboard component fullscreen.
   *
   * @param component the component to make fullscreen.
   * @return the fullscreen component.
   */
  // TODO
  public static ShuffleboardComponent makeFullscreen(ShuffleboardComponent component) {
    return component.withPosition(0, 0).withSize(10, 4);
  }

  /**
   * Adds swerve module states to a Shuffleboard tab.
   *
   * @param tab the Shuffleboard tab to add the swerve module states to.
   * @param title the title of the swerve module states.
   * @param swerveModuleStatesSupplier a supplier for swerve module states.
   * @return the swerve module states widget.
   */
  public static SuppliedValueWidget<double[]> addSwerveModuleStates(
      ShuffleboardTab tab, String title, Supplier<SwerveModuleState[]> swerveModuleStatesSupplier) {
    return tab.addDoubleArray(
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
   * Gets the Network Tables table.
   *
   * @param table the name of the Network Tables table.
   * @return the Network Tables table.
   */
  private static NetworkTable getTable(String table) {
    return NetworkTableInstance.getDefault().getTable(table);
  }

  /**
   * Adds a boolean entry to a Network Tables table.
   *
   * @param table the name of a Network Tables table.
   * @param name the name of the boolean entry.
   * @param defaultValue the default value of the entry.
   * @return the boolean entry.
   */
  public static BooleanEntry addBooleanEntry(String table, String name, boolean defaultValue) {
    BooleanEntry entry = getTable(table).getBooleanTopic(name).getEntry(defaultValue);

    entry.set(defaultValue);

    return entry;
  }

  /**
   * Adds a boolean entry to the Network Tables table.
   *
   * @param table the name of a Network Tables table.
   * @param name the name of the boolean entry.
   * @return the boolean entry.
   */
  public static BooleanEntry addBooleanEntry(String table, String name) {
    return addBooleanEntry(table, name, false);
  }

  /**
   * Adds a double entry to the Network Tables table.
   *
   * @param table the name of a Network Tables table.
   * @param name the name of the boolean entry.
   * @param defaultValue the default value of the entry.
   * @return the double entry.
   */
  public static DoubleEntry addDoubleEntry(String table, String name, double defaultValue) {
    DoubleEntry entry = getTable(table).getDoubleTopic(name).getEntry(defaultValue);

    entry.set(defaultValue);

    return entry;
  }

  /**
   * Adds a double entry to the Network Tables table.
   *
   * @param table the name of a Network Tables table.
   * @param name the name of the boolean entry.
   * @return the double entry.
   */
  public static DoubleEntry addDoubleEntry(String table, String name) {
    return addDoubleEntry(table, name, 0.0);
  }
}
