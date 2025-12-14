package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Robot;

@Autonomous(name = "PIDAuto")
public class PIDAuto extends OpMode {
    ElapsedTime driveTimer = new ElapsedTime();
    protected Robot robot;

    @Override
    /* Code to run ONCE when the driver hits INIT
     *
     * Initializes Robot object.
     */
    public void init(){
        robot = new Robot();
        robot.init(hardwareMap);

        robot.odo.resetPosAndIMU();
        robot.odometry.resetYaw();
        telemetry.addLine("Initialized");
    }

    /* Code to run REPEATEDLY after the driver hits INIT, but before they hit START.
     *
     * As of right now, I have nothing that needs to be done in the initialization loop, but in the
     * future that might be useful, so here the architecture is set up and ready to be used.
     */
    @Override
    public void init_loop(){
        robot.init_loop();
    }

    /* Code to run ONCE when the driver hits START
     */
    @Override
    public void start(){
        robot.drivetrain.setPIDDriveActive(true);
        robot.drivetrain.setTargetPosition(0, 48, 0, 1, 1);
        driveTimer.reset();
    }

    @Override
    public void loop(){
        robot.loop();
        robot.addTelemetry(telemetry);
    }
}
