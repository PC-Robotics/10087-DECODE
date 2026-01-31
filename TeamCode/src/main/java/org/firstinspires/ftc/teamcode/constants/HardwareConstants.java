package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class HardwareConstants {
    /*
     * Declaring constants for the open and closed positions of the servo.
     */
    public static final double CLAW_OPEN = 0.20; // Claw position closed
    public static final double CLAW_CLOSE = 0.3; // Claw position open

    /*
     * Declaring constants for the up, middle, and down positions of the servo.
     */
    public static final double ELEVATOR_UP = 0.7; // Elevator position at maximum height
    public static final double ELEVATOR_MID = 0.55; // Elevator position at middle height changed position to .55 from .5
    public static final double ELEVATOR_DOWN = 0.2; // Elevator position at minimum height

    /*
     * When we control our launcher motor, we are using encoders. These allow the control system
     * to read the current speed of the motor and apply more or less power to keep it at a constant
     * velocity. Here we are setting the target, and minimum velocity that the launcher should run
     * at. The minimum velocity is a threshold for determining when to fire.
     */
    public static final double FLYWHEEL_TARGET_VELOCITY = 2000; // old 1125
    public static final double FLYWHEEL_MIN_VELOCITY = 1990; // old: 2300

    /*
     * Set the odometry pod positions relative to the point that the odometry computer tracks around.
     * The X pod offset refers to how far sideways from the tracking point the X (forward) odometry
     * pod is. Left of the center is a positive number, right of center is a negative number.
     * the Y pod offset refers to how far forwards from the tracking point the Y (strafe) odometry
     * pod is. forward of center is a positive number, backwards is a negative number.
     */
    public static double X_OFFSET = 120;
    public static double Y_OFFSET = 96;

    public static final double FEED_TIME_SECONDS = 1; // Time we wait before lowering elevator after launching
    public static final double LOWER_TIME_SECONDS = 1.6;
}
