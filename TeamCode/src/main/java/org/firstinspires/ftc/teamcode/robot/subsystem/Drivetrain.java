package org.firstinspires.ftc.teamcode.robot.subsystem;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Subsystem;

/*
 * This class handles everything relating to the 4 motors that spin the mecanum wheels on the robot
 * that are used to move the robot around.
 */
public class Drivetrain extends Subsystem {
    /*
     * Declaring a variable for each drive wheel to make it easier to manipulate and use.
     */
    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;
    /*
     * Declaring a variable used to keep track of the rotation of the robot which is needed to use
     * field centric controls.
     */
    double heading;

    double strafe;
    double forward;

    /*
     * Using the parent class, Subsystem, to construct Drivetrain.
     */
    public Drivetrain(Robot robot) {
        super(robot);
    }
    /*
     * Returns the heading of the robot in radians.
     */
    public double getHeading(){
        return heading;
    }
    /*
     * The drive function here uses the mecanum wheel magic that I don't really get the math behind
     * to move around the robot and drive.
     */
    public void drive(double y, double x, double rotate){
        strafe = x;
        forward = y;
        /* the denominator is the largest motor power (absolute value) or 1
         * This ensures all the powers maintain the same ratio,
         * but only if at least one is out of the range [-1, 1]
         */
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);

        leftFrontPower = (forward + strafe + rotate) / denominator;
        leftBackPower = (forward - strafe + rotate) / denominator;
        rightFrontPower = (forward - strafe - rotate) / denominator;
        rightBackPower = (forward + strafe - rotate) / denominator;

        robot.leftFrontDrive.setPower(leftFrontPower);
        robot.rightFrontDrive.setPower(rightFrontPower);
        robot.leftBackDrive.setPower(leftBackPower);
        robot.rightBackDrive.setPower(rightBackPower);

    }
    public void fieldCentricDrive(double y, double x, double rotate){
        strafe = x * Math.cos(-heading) - y * Math.sin(-heading);
        forward = x * Math.sin(-heading) + y * Math.cos(-heading);

        /* the denominator is the largest motor power (absolute value) or 1
         * This ensures all the powers maintain the same ratio,
         * but only if at least one is out of the range [-1, 1]
         */
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);

        leftFrontPower = (forward + strafe + rotate) / denominator;
        leftBackPower = (forward - strafe + rotate) / denominator;
        rightFrontPower = (forward - strafe - rotate) / denominator;
        rightBackPower = (forward + strafe - rotate) / denominator;

        robot.leftFrontDrive.setPower(leftFrontPower);
        robot.rightFrontDrive.setPower(rightFrontPower);
        robot.leftBackDrive.setPower(leftBackPower);
        robot.rightBackDrive.setPower(rightBackPower);
    }
    /*
     * Method for resetting the yaw on a button press or something.
     */
    public void resetYaw(){
        robot.imu.resetYaw();
    }
    /*
     * Updating the heading of the robot every frame.
     */
    @Override
    public void loop(){
        heading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        addToTelemetry("Robot heading (degrees)", (int)Math.toDegrees(heading));
        addToTelemetry("Strafe power", strafe);
        addToTelemetry("Forward power", forward);
        addToTelemetry("Left Front", leftFrontPower);
        addToTelemetry("Right Front", rightFrontPower);
        addToTelemetry("Left Back", leftBackPower);
        addToTelemetry("Right Back", rightBackPower);
    }
}
