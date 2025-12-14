package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Field Centric Driving", group = "Teleop")
@Disabled
public class FieldCentricTeleop extends Teleop {
    @Override
    public void init(){
        super.init();
        isFieldCentric = true;
    }
}
