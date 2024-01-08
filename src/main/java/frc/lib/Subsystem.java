package frc.lib;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Base class for all subsystems. Includes Shuffleboard tab requirement.
 */
public abstract class Subsystem extends SubsystemBase {

    /**
     * Adds information about a subsystem to a Shuffleboard tab.
     * 
     * @param tab the Shuffleboard tab to add information to.
     */
    public abstract void addToShuffleboard(ShuffleboardTab tab);

}
