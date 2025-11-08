package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Field Centric Driving", group = "Teleop")
public class FieldCentricTeleop extends Teleop {
    @Override
    public void init(){
        super.init();
        isFieldCentric = true;
    }
}
