package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Sam's Teleop", group = "Teleop")
public class SamsTeleop extends Teleop {
    @Override
    public void init(){
        super.init();
        isFieldCentric = true;
        openClawAfterShooting = true;
        autoStopIntake = false;
        telemetry.addData("Driver Profile Selected", "Sam");
    }
}