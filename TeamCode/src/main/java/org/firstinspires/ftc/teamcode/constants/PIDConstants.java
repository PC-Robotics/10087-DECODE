package org.firstinspires.ftc.teamcode.constants;

public class PIDConstants {
    // Constants dealing with moving forward and backward
    public static double DRIVE_KP = 0.36;
    public static double DRIVE_KI = 0;
    public static double DRIVE_KD = 0;
    public static final double DRIVE_TOLERANCE = 0.5;
    public static final double DRIVE_DEADBAND = 0.25; // must be less than tolerance
    public static final double DRIVE_MAX_AUTO = 0.8;


    // Constants dealing with moving side to side
    public static double STRAFE_KP = 0.5;//0.5
    public static double STRAFE_KI = 0;
    public static double STRAFE_KD = 0;
    public static final double STRAFE_TOLERANCE = 0.5;
    public static final double STRAFE_DEADBAND = 0.25; //must be less than tolerance
    public static final double STRAFE_MAX_AUTO = 0.8;


    // Constants dealing with turning
    public static double YAW_KP = 0.1;//0.5
    public static double YAW_KI = 0;
    public static double YAW_KD = 0;
    public static final double YAW_TOLERANCE = 3;//1
    public static final double YAW_DEADBAND = .25;
    public static final double YAW_MAX_AUTO = 0.8;
}
