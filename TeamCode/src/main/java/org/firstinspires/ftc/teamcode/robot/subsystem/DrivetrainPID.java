package org.firstinspires.ftc.teamcode.robot.subsystem;

import static org.firstinspires.ftc.teamcode.constants.PIDConstants.*;
import static org.firstinspires.ftc.teamcode.constants.PIDConstants.DRIVE_DEADBAND;
import static org.firstinspires.ftc.teamcode.constants.PIDConstants.DRIVE_KD;
import static org.firstinspires.ftc.teamcode.constants.PIDConstants.DRIVE_KI;
import static org.firstinspires.ftc.teamcode.constants.PIDConstants.DRIVE_KP;
import static org.firstinspires.ftc.teamcode.constants.PIDConstants.DRIVE_MAX_AUTO;
import static org.firstinspires.ftc.teamcode.constants.PIDConstants.DRIVE_TOLERANCE;
import static org.firstinspires.ftc.teamcode.constants.PIDConstants.STRAFE_DEADBAND;
import static org.firstinspires.ftc.teamcode.constants.PIDConstants.STRAFE_KD;
import static org.firstinspires.ftc.teamcode.constants.PIDConstants.STRAFE_KI;
import static org.firstinspires.ftc.teamcode.constants.PIDConstants.STRAFE_KP;
import static org.firstinspires.ftc.teamcode.constants.PIDConstants.STRAFE_MAX_AUTO;
import static org.firstinspires.ftc.teamcode.constants.PIDConstants.STRAFE_TOLERANCE;
import static org.firstinspires.ftc.teamcode.constants.PIDConstants.YAW_DEADBAND;
import static org.firstinspires.ftc.teamcode.constants.PIDConstants.YAW_KD;
import static org.firstinspires.ftc.teamcode.constants.PIDConstants.YAW_KI;
import static org.firstinspires.ftc.teamcode.constants.PIDConstants.YAW_KP;
import static org.firstinspires.ftc.teamcode.constants.PIDConstants.YAW_MAX_AUTO;
import static org.firstinspires.ftc.teamcode.constants.PIDConstants.YAW_TOLERANCE;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robot.PIDController;
import org.firstinspires.ftc.teamcode.robot.Robot;

import java.util.concurrent.TimeUnit;

public class DrivetrainPID extends Drivetrain {
    private boolean PIDDriveActive = false;
    private boolean holding;
    private ElapsedTime holdTimer = new ElapsedTime();
    private double holdTime;

    private PIDController driveController = new PIDController(
            DRIVE_KP, DRIVE_KI, DRIVE_KD, DRIVE_MAX_AUTO, DRIVE_TOLERANCE, DRIVE_DEADBAND,
            false
    );
    private PIDController strafeController = new PIDController(
            STRAFE_KP, STRAFE_KI, STRAFE_KD, STRAFE_MAX_AUTO, STRAFE_TOLERANCE, STRAFE_DEADBAND,
            false
    );
    private PIDController yawController = new PIDController(
            YAW_KP, YAW_KI, YAW_KD, YAW_MAX_AUTO, YAW_TOLERANCE, YAW_DEADBAND,
            true
    );

    private double xPosition;
    private double yPosition;
    private double headingDeg;
    private double headingRad;

    private double targetX;
    private double targetY;
    private double targetHeading;

    /*
     * Using the parent class, Subsystem, to construct DrivetrainPID.
     */
    public DrivetrainPID(Robot robot) {
        super(robot);
    }
    public void setPIDDriveActive(boolean active){
        PIDDriveActive = active;
    }

    public void updatePosition(){
        xPosition = robot.odometry.getXPosition(DistanceUnit.INCH);
        yPosition = robot.odometry.getYPosition(DistanceUnit.INCH);
        headingDeg = robot.odometry.getHeading(AngleUnit.DEGREES);
        headingRad = robot.odometry.getHeading(AngleUnit.RADIANS);
    }
    public void updateDrive(){
        if (holding) {
            double xDistance = targetX - xPosition;
            double yDistance = targetY - yPosition;

            double rotX = xDistance * Math.cos(-headingRad) - yDistance * Math.sin(-headingRad);
            double rotY = xDistance * Math.sin(-headingRad) + yDistance * Math.cos(-headingRad);

            double strafe = driveController.getOutputFromError(rotX);
            double forward = strafeController.getOutputFromError(rotY);
            double rotate = yawController.getOutput(headingDeg);

            drive(forward, -strafe, -rotate);

            if (driveController.isInPosition() && strafeController.isInPosition() && yawController.isInPosition()) {
                if (holdTimer.time(TimeUnit.SECONDS) > holdTime) {
                    holding = false;
                }
            } else {
                holdTimer.reset();
            }
        }
    }
    public void setTargetPosition(double yLocation, double xLocation, double headingDegrees, double power, double holdTimeSeconds){
        driveController.reset(yLocation, power);
        strafeController.reset(xLocation, power);
        yawController.reset(headingDegrees, power);

        targetY = yLocation;
        targetX = xLocation;
        targetHeading = headingDegrees;

        holdTime = holdTimeSeconds;
        holding = true;
        holdTimer.reset();
    }

    @Override
    public void init(){
        updatePosition();
        setTargetPosition(yPosition, xPosition, headingDeg, 0, 1);
    }
    @Override
    public void loop() {
        if (PIDDriveActive) {
            updatePosition();
            updateDrive();
        } else {
            super.loop();
        }
        addToTelemetry("Holding", holding);
        addToTelemetry("Target X", targetX);
        addToTelemetry("Target Y", targetY);
        addToTelemetry("Target heading", targetHeading);

        addToTelemetry("Position X", xPosition);
        addToTelemetry("Position Y", yPosition);
        addToTelemetry("Heading degrees", headingDeg);
    }
}
