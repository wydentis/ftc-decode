package org.firstinspires.ftc.team28420.module.shooter;

import org.firstinspires.ftc.team28420.config.ShooterConf;

import java.util.Map;
import java.util.TreeMap;

/**
 * A lookup table for linear interpolation between data points.
 */
public class InterpolationTable {
    private final TreeMap<Double, Double> table = new TreeMap<>();

    /**
     * Adds a data point to the table.
     *
     * @param distance input distance
     * @param velocity target velocity
     * @return this instance for method chaining
     */
    public InterpolationTable addPoint(double distance, double velocity) {
        table.put(distance, velocity);
        return this;
    }

    /**
     * Returns an interpolarted value for the given distance.
     *
     * @param distance input distance to look up
     * @return interpolated velocity or edge value if out of bounds
     */
    public double getInterpolatedValue(double distance) {
        if (table.isEmpty()) return ShooterConf.MIN_SPEED;

        Double exactValue = table.get(distance);
        if (exactValue != null) return exactValue;

        Map.Entry<Double, Double> lowerEntry = table.lowerEntry(distance);
        Map.Entry<Double, Double> upperEntry = table.higherEntry(distance);

        if (lowerEntry == null) return getSafeValue(upperEntry);
        if (upperEntry == null) return getSafeValue(lowerEntry);

        double x0 = lowerEntry.getKey();
        double y0 = lowerEntry.getValue();
        double x1 = upperEntry.getKey();
        double y1 = upperEntry.getValue();

        return interpolate(distance, x0, x1, y0, y1);
    }

    /**
     * Extracts a value from a map entry safely.
     *
     * @param entry map entry to process
     * @return entry value or minimum speed if null
     */
    private double getSafeValue(Map.Entry<Double, Double> entry) {
        if (entry == null || entry.getValue() == null) {
            return ShooterConf.MIN_SPEED;
        }
        return entry.getValue();
    }

    /**
     * Performs linear interpolation calculation.
     *
     * @param x  target x
     * @param x0 lower x
     * @param x1 upper x
     * @param y0 lower y
     * @param y1 upper y
     * @return interpolated y value
     */
    private double interpolate(double x, double x0, double x1, double y0, double y1) {
        return y0 + (x - x0) * (y1 - y0) / (x1 - x0);
    }
}