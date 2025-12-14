package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Kai's Teleop", group = "Teleop")
public class KaisTeleop extends Teleop {
    @Override
    public void init(){
        super.init();
        isFieldCentric = true;
        openClawAfterShooting = false;
        autoStopIntake = true;
        telemetry.addData("Driver Profile Selected", "Kai");
    }
}