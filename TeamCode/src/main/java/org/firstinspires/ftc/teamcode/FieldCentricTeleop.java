package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Field Centric Driving", group = "Teleop")
public class FieldCentricTeleop extends Teleop {
    @Override
    public void loop(){
        super.loop();
        robot.drivetrain.fieldCentricDrive(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x
        );
    }
}
