package org.firstinspires.ftc.teamcode.robot.subsystem;

import org.firstinspires.ftc.teamcode.constants.HardwareConstants;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Subsystem;

/*
 * This class handles everything relating to the two 6000 rpm motors that spin the flywheels on the
 * robot that launch the artifacts.
 */
public class Flywheels extends Subsystem {
    /*
     * Boolean value to keep track of whether or not the flywheels are running.
     */
    boolean flywheelsRunning;

    /*
     * Using the parent class, Subsystem, to construct Flywheels.
     */
    public Flywheels(Robot robot) {
        super(robot);
    }

    /*
     * Returns true or false depending on whether the flywheels are running at sufficient speed to
     * launch an artifact. Due to strange hardware issues where only the left flywheel was reading
     * the correct velocity, we are only checking the left flywheel which has seemed good enough
     * but should be fixed.
     */
    public boolean flywheelsReady(){
        return (-robot.leftFlywheel.getVelocity() > HardwareConstants.FLYWHEEL_MIN_VELOCITY);
    }
    /*
     * Method for activating / deactivating the flywheels.
     */
    public void setFlywheels(boolean on){
        flywheelsRunning = on;
        if (on){
            robot.leftFlywheel.setVelocity(HardwareConstants.FLYWHEEL_TARGET_VELOCITY);
            robot.rightFlywheel.setVelocity(HardwareConstants.FLYWHEEL_TARGET_VELOCITY);
        } else {
            robot.leftFlywheel.setVelocity(0);
            robot.rightFlywheel.setVelocity(0);
        }
    }
    /*
     * Method for toggling the flywheels to whatever they aren't currently set to. If this is being
     * called from a teleop, use gamepad.buttonWasPressed() rather than gamepad.button, as one
     * button press will last multiple frames and immediately toggle off the motors.
     */
    public void toggleFlywheels(){
        setFlywheels(!flywheelsRunning);
    }
    /*
     * Initializing the flywheels to off because it's not like they would ever start running anyways.
     */
    @Override
    public void init(){
        flywheelsRunning = false;
    }
    @Override
    public void loop(){
        addToTelemetry("Flywheels running", flywheelsRunning);
        addToTelemetry("Left flywheel speed", robot.leftFlywheel.getVelocity());
        addToTelemetry("Right flywheel speed", robot.rightFlywheel.getVelocity());
        addToTelemetry("Flywheels ready", flywheelsReady());
    }
}
