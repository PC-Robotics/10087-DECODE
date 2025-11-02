package org.firstinspires.ftc.teamcode.robot.subsystem;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Subsystem;

public class DrivetrainFieldCentric extends Subsystem {
    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;

    private double botHeading;

    public DrivetrainFieldCentric(Robot robot) {
        super(robot);
    }

    public double heading(){
        return botHeading;
    }

    public void drive(double forward, double strafe, double rotate){

        /* the denominator is the largest motor power (absolute value) or 1
         * This ensures all the powers maintain the same ratio,
         * but only if at least one is out of the range [-1, 1]
         */
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);

        leftFrontPower = (forward + strafe + rotate) / denominator;
        rightFrontPower = (forward - strafe - rotate) / denominator;
        leftBackPower = (forward - strafe + rotate) / denominator;
        rightBackPower = (forward + strafe - rotate) / denominator;

        robot.leftFrontDrive.setPower(leftFrontPower);
        robot.rightFrontDrive.setPower(rightFrontPower);
        robot.leftBackDrive.setPower(leftBackPower);
        robot.rightBackDrive.setPower(rightBackPower);

    }

    @Override
    public void loop(){
        botHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }
}
