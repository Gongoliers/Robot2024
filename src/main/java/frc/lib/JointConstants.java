package frc.lib;

import edu.wpi.first.math.system.plant.DCMotor;

public record JointConstants(
    double mass,
    double length,
    double radius,
    double moi,
    double gearing,
    DCMotor motor,
    int motorCount) {}
