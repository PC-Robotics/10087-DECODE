package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class PIDConstants {
    // Constants dealing with moving forward and backward
    public static double DRIVE_DEADBAND = 0.25;
    public static double DRIVE_KD = 0.025;
    public static double DRIVE_KI = 0;
    public static double DRIVE_KP = 0.2;
    public static double DRIVE_MAX_AUTO = 0.8;
    public static double DRIVE_TOLERANCE = 0.5;
    public static double STRAFE_DEADBAND = 0.25;
    public static double STRAFE_KD = 0.025;
    public static double STRAFE_KI = 0;
    public static double STRAFE_KP = 0.2;
    public static double STRAFE_MAX_AUTO = 0.8;
    public static double STRAFE_TOLERANCE = 0.5;
    public static double YAW_DEADBAND = 0.25;
    public static double YAW_KD = 0.0035;
    public static double YAW_KI = 0;
    public static double YAW_KP = 0.045;
    public static double YAW_MAX_AUTO = 0.8;
    public static double YAW_TOLERANCE = 3;

}
