package frc.lib;

import edu.wpi.first.math.geometry.Rotation2d;

public class SnapRotation {

  private final Rotation2d multiple;

  private SnapRotation(Rotation2d multiple) {
    this.multiple = multiple;
  }

  public static SnapRotation to(Rotation2d multiple) {
    return new SnapRotation(multiple);
  }

  private double snapToNearest(double n, double multiple) {
    return Math.round(n / multiple) * multiple;
  }

  public Rotation2d snap(Rotation2d angle) {
    double snapped = snapToNearest(angle.getRotations(), multiple.getRotations());

    return Rotation2d.fromRotations(snapped);
  }
}
