package frc.lib;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.util.Color;

public class InterpolatableColor {

    private final Color zero, one;

    public InterpolatableColor(Color zero, Color one) {
        this.zero = zero;
        this.one = one;
    }

    public Color sample(double t) {
        t = MathUtil.clamp(t, 0, 1);

        double r = zero.red * (1 - t) + one.red * t;
        double g = zero.green * (1 - t) + one.green * t;
        double b = zero.blue * (1 - t) + one.blue * t;
        
        return new Color(r, g, b);
    }

    public Color sample(double x, double min, double max) {
        double t = x / (max - min);

        return sample(t);
    }

}
