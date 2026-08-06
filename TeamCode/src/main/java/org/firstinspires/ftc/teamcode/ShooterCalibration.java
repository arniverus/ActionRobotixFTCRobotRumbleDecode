package org.firstinspires.ftc.teamcode;

/**
 * Calibration class for Shooter Flywheel and Hood.
 * Use Desmos Regression to find your A, B, and C constants.
 */
public class ShooterCalibration {

    // ========================================================
    // FLYWHEEL CALIBRATION (Quadratic: ax^2 + bx + c)
    // ========================================================
    // Replace these with your values from Desmos
    private static final double FW_A = 0.025;
    private static final double FW_B = 1.15;
    private static final double FW_C = 0.45; // Base power/velocity

    // ========================================================
    // HOOD CALIBRATION (Quadratic or Linear)
    // ========================================================
    // Replace these with your values from Desmos
    private static final double HOOD_A = 0.0001;
    private static final double HOOD_B = 0.005;
    private static final double HOOD_C = 0.2; // Base servo position

    /**
     * Calculates the required Flywheel Power based on distance.
     * @param distance Distance to goal from Pedro Follower
     * @return Motor power clamped between 0 and 1
     */
    public static double getFlywheelPower(double distance) {
        double power = (FW_A * Math.pow(distance, 2)) + (FW_B * distance) + FW_C;
        return Math.max(0.0, Math.min(1.0, power));
    }

    /**
     * Calculates the required Hood Servo Position based on distance.
     * @param distance Distance to goal from Pedro Follower
     * @return Servo position clamped between 0 and 1
     */
    public static double getHoodPosition(double distance) {
        double position = (HOOD_A * Math.pow(distance, 2)) + (HOOD_B * distance) + HOOD_C;
        return Math.max(0.0, Math.min(1.0, position));
    }
}