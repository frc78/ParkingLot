// Copyright (c) Team 78!

package frc.robot;

/** A utility class containing widely used functions*/
public class Utility {
    /**
     * Takes in the number of encoder clicks, and converst it to meters
     * @param clicks The # of encoder clicks
     * @return Meters
     */
    public static double encodersToMeters(double clicks) {
        return ((clicks / Constants.UNITS_PER_REVOLUTION) / Constants.WHEEL_GEAR_RATIO) * Constants.WHEEL_CIRC_METERS;
    }

    /**
     * Takes in the number of meters, and converst it to encoder clicks
     * @param clicks The # of encoder meters
     * @return Encoder clicks
     */
    public static double metersToEncoders(double meters) {
        return ((meters / Constants.WHEEL_CIRC_METERS) * Constants.WHEEL_GEAR_RATIO) * Constants.UNITS_PER_REVOLUTION;
    }
}
