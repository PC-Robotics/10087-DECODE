package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.support.ConstantsPID.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.support.PIDController;

public class DriveBasePID extends DriveBaseOdometry
{
    private ElapsedTime holdTimer = new ElapsedTime();

    private PIDController driveController = new PIDController(DRIVE_KP,DRIVE_KI,DRIVE_KD,DRIVE_MAX_AUTO,DRIVE_TOLERANCE,DRIVE_DEADBAND,false);
    private PIDController strafeController = new PIDController(STRAFE_KP,STRAFE_KI,STRAFE_KD,STRAFE_MAX_AUTO,STRAFE_TOLERANCE,STRAFE_DEADBAND, false);
    private PIDController yawController = new PIDController(YAW_KP,YAW_KI,YAW_KD,YAW_MAX_AUTO,YAW_TOLERANCE,YAW_DEADBAND,true);

    public DriveBasePID(LinearOpMode mode,boolean isFC)
    {
        super(mode,isFC);
    }

    public void init()
    {
        super.init();
    }

    public void forward(double distanceInches, double power, double holdTime)
    {
        driveController.reset(getYPosition(DistanceUnit.INCH)+distanceInches,power);
        strafeController.reset(getXPosition(DistanceUnit.INCH));
        yawController.reset();
        holdTimer.reset();

        while (myOpMode.opModeIsActive())
        {
            updatePositionAndTelemetry();

            drive(-driveController.getOutput(getYPosition(DistanceUnit.INCH)),-strafeController.getOutput(getXPosition(DistanceUnit.INCH)), yawController.getOutput(getHeading(AngleUnit.DEGREES)));

            if(driveController.isInPosition() && yawController.isInPosition())
            {
                if(holdTimer.time() > holdTime)  break;
            }
            else holdTimer.reset();

            myOpMode.sleep(10);
        }

        drive(0,0,0);
    }

    public void strafe(double distanceInches, double power, double holdTime)
    {
        driveController.reset(getYPosition(DistanceUnit.INCH));
        strafeController.reset(getXPosition(DistanceUnit.INCH)+distanceInches,power);
        yawController.reset();
        holdTimer.reset();

        while (myOpMode.opModeIsActive())
        {
            updatePositionAndTelemetry();

            drive(-driveController.getOutput(getYPosition(DistanceUnit.INCH)),-strafeController.getOutput(getXPosition(DistanceUnit.INCH)), yawController.getOutput(getHeading(AngleUnit.DEGREES)));

            if(strafeController.isInPosition() && yawController.isInPosition())
            {
                if(holdTimer.time() > holdTime)  break;
            }
            else holdTimer.reset();

            myOpMode.sleep(10);
        }

        drive(0,0,0);
    }

    public void turnTo(double headingDegree, double power, double holdTime)
    {
        yawController.reset(headingDegree,power);
        while(myOpMode.opModeIsActive())
        {
            updatePositionAndTelemetry();

            drive(0,0,yawController.getOutput(getHeading(AngleUnit.DEGREES)));

            if(yawController.isInPosition())
            {
                if(holdTimer.time() > holdTime) break;
            }
            else holdTimer.reset();
            // Test if needed
            myOpMode.sleep(10);
        }

        // Test if needed
        drive(0,0,0);
    }

    public void goToPosition(double yLocation, double xLocation, double headingDegree, double power, double holdTime)
    {
        driveController.reset(yLocation, power);
        strafeController.reset(xLocation, power);
        yawController.reset(headingDegree, power);

        while(myOpMode.opModeIsActive())
        {
            updatePositionAndTelemetry();

            drive(-driveController.getOutput(getYPosition(DistanceUnit.INCH)),-strafeController.getOutput(getXPosition(DistanceUnit.INCH)), yawController.getOutput(getHeading(AngleUnit.DEGREES)));

            if(driveController.isInPosition() && strafeController.isInPosition() && yawController.isInPosition())
            {
                if(holdTimer.time() > holdTime) break;
            }
            else holdTimer.reset();

            myOpMode.sleep(10);
        }

        drive(0,0,0);
    }
}
