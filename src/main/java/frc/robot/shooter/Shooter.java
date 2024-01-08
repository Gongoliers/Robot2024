package frc.robot.shooter;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.lib.Subsystem;

/**
 * The subsystem class for the shooter subsystem.
 */
public class Shooter extends Subsystem {

    /**
     * The instance variable for the shooter subsystem singleton. 
     */
    private static Shooter instance = null;
   
    /**
     * Creates a new instance of the shooter subsystem.
     */
    private Shooter() {}

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
        // TODO
    }

    @Override
    public void addToShuffleboard(ShuffleboardTab tab) {
        // TODO
    }

}
