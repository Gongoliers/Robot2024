package frc.lib;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import java.util.HashMap;

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
}
