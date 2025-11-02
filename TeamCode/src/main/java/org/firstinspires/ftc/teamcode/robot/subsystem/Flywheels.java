package org.firstinspires.ftc.teamcode.robot.subsystem;

import org.firstinspires.ftc.teamcode.DriveBase;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Subsystem;

public class Flywheels extends Subsystem {
    /*
     * When we control our launcher motor, we are using encoders. These allow the control system
     * to read the current speed of the motor and apply more or less power to keep it at a constant
     * velocity. Here we are setting the target, and minimum velocity that the launcher should run
     * at. The minimum velocity is a threshold for determining when to fire.
     */
    final double FLYWHEEL_TARGET_VELOCITY = 1125;
    final double FLYWHEEL_MIN_VELOCITY = 2300;

    boolean flywheelsRunning;
    public Flywheels(Robot robot) {
        super(robot);
    }

    public void setFlywheels(boolean on){
        flywheelsRunning = on;
        if (on){
            robot.leftFlywheel.setVelocity(FLYWHEEL_TARGET_VELOCITY);
            robot.leftFlywheel.setVelocity(FLYWHEEL_TARGET_VELOCITY);
        } else {
            robot.leftFlywheel.setVelocity(0);
            robot.rightFlywheel.setVelocity(0);
        }
    }
    public void toggleFlywheels(){
        setFlywheels(!flywheelsRunning);
    }

    public boolean flywheelsReady(){
        if (-robot.leftFlywheel.getVelocity() > FLYWHEEL_MIN_VELOCITY) {
            return true;
        } else return false;
    }

    @Override
    public void init(){
        flywheelsRunning = false;
    }
}
