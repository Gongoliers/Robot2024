package frc.robot.lights;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.util.Color;
import frc.lib.Subsystem;
import frc.lib.Telemetry;
import java.util.Map;

/** Subsystem class for the lights subsystem.  */
public class Lights extends Subsystem {

    /** Instance variable for the lights subsystem singleton  */
    private static Lights instance = null;

    private SimpleWidget mainLightWidget;

    /** Creates a new instance of the lights subsystem.  */
    private Lights() {}

    /**
     * Gets the instance of the lights subsystem.
     * 
     * @return the instance of the lights subsystem.
     */
    public static Lights getInstance() {
        if (instance == null) {
            instance = new Lights();
        }

        return instance;
    }

    @Override
    public void periodic() {}

    @Override
    public void addToShuffleboard(ShuffleboardTab tab) {
        ShuffleboardLayout mainColumn = Telemetry.addColumn(tab, "Main Light");

        mainLightWidget = mainColumn.add("Main Light Color", true).withProperties(Map.of("colorWhenFalse", "#000000", "colorWhenTrue", getColor().toHexString()));
    }

    public Color getColor() {
        return new Color();
    }
    
}
