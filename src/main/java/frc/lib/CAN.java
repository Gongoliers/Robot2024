package frc.lib;

import java.util.Objects;

/**
 * Record class representing a CAN identifier for a device.
 */
public record CAN(int id, String bus) {

    /**
     * Creates a CAN identifier for a device.
     * 
     * @param id the numeric identifier assigned to the device.
     * @param bus the name of the CAN bus that the device is located on.
     */
    public CAN {
        Objects.requireNonNull(id);
        Objects.requireNonNull(bus);
    }

    /**
     * Creates a CAN identifier for a device.
     * 
     * @param id the numeric identifier assigned to the device.
     */
    public CAN(int id) {
        this(id, "");
    }
}
