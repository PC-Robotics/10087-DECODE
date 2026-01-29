package org.firstinspires.ftc.teamcode.robot.subsystem;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "PIDF", group = "Teleop")
public class PIDFTuner extends OpMode {
    public DcMotorEx flyWheelMotor;

    public double highVelocity = 2000;

    public double lowVelocity = 1800;

    double curTargetVelocity = highVelocity;

    double F = 0;

    double P = 0;

    double[] stepSizes = {10.0 , 1.0 , 0.1, 0.001, 0.0001};

    int stepIndex = 1;

    @Override
    public void init() {
        flyWheelMotor = hardwareMap.get(DcMotorEx.class, "right_launcher");
        flyWheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flyWheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);
        flyWheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);
        telemetry.addLine("Init Complete");
    }

    @Override
    public void loop() {

        if (gamepad1.squareWasPressed()){
            if(curTargetVelocity == highVelocity){
                curTargetVelocity = lowVelocity;
            } else {
                curTargetVelocity = highVelocity;

            }
        }
        if (gamepad1.crossWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpadLeftWasPressed()){
            F -= stepSizes[stepIndex];
        }
        if (gamepad1.dpadRightWasPressed()){
            F += stepSizes[stepIndex];
        }

        if (gamepad1.dpadUpWasPressed()){
            P -= stepSizes[stepIndex];
        }
        if (gamepad1.dpadDownWasPressed()){
            P += stepSizes[stepIndex];
        }

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);
        flyWheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);

        flyWheelMotor.setVelocity(curTargetVelocity);

        double curVelocity = flyWheelMotor.getVelocity();
        double error = curTargetVelocity - curVelocity;

        telemetry.addData("TargetVelocity:" , curTargetVelocity);
        telemetry.addData("CurrentVelocity:" , "%2f", curVelocity);
        telemetry.addData("Error:" , "%2f", error);
        telemetry.addLine("---------------------");
        telemetry.addData("Tuning P", "%.4f (Dpad U/D)", P);
        telemetry.addData("Tuning F", "%.4f (Dpad L/R)", F);
        telemetry.addData("Step Size", "%.4f (B button)",stepSizes[stepIndex] );



    }
}