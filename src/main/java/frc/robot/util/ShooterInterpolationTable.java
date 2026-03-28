package frc.robot.util;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterInterpolationTable {
    // Math map mapping distance (meters) to required wheel RPS
    public static final InterpolatingDoubleTreeMap map = new InterpolatingDoubleTreeMap();

    static {
        // Populate with empirical values from successful shots on the field.
        // FORMAT: Distance (Meters), RPS (Revolutions/Seconds).
        map.put(1.0, 30.0);
        map.put(2.0, 45.0);
        map.put(3.0, 55.0);
        map.put(4.0, 65.0);
        map.put(5.0, 75.0);
    }

    /**
     * Gets the interpolated RPS for a given distance to the target.
     * 
     * @param distance The distance to the target in meters.
     * @return The interpolated RPS to shoot at. Returns 0 if distance is 0 or less.
     */
    public static double getRPS(double distance) {
        if (distance <= 0) {
            return 0.0;
        }
        return map.get(distance);
    }
}
