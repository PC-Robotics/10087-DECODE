package org.firstinspires.ftc.teamcode.robot.subsystem;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Subsystem;

/*
 * This class handles everything related to the two continuous rotation servos on the front of the
 * robot that spin intake wheels to suck in the artifacts.
 */
public class Intakes extends Subsystem {
    /*
     * Boolean to keep track of whether or not the intakes are running
     */
    private boolean intakesRunning;
    /*
     * Using the parent class, Subsystem, to construct Claw.
     */
    public Intakes(Robot robot) {
        super(robot);
    }
    /*
     * Returns true or false depending on whether or not the intakes are running.
     */
    public boolean intakesRunning(){
        return intakesRunning;
    }
    /*
     * Method for activating / deactivating the intakes.
     */
    public void setIntakesRunning(boolean on){
        intakesRunning = on;
        if (on){
            robot.leftIntake.setPower(1);
            robot.rightIntake.setPower(1);
        } else {
            robot.leftIntake.setPower(0);
            robot.rightIntake.setPower(0);
        }
    }
    /*
     * Method for toggling the intakes to whatever they aren't currently set to. If this is being
     * called from a teleop, use gamepad.buttonWasPressed() rather than gamepad.button, as one
     * button press will last multiple frames and immediately toggle off the motors.
     */
    public void toggleIntakes(){
        setIntakesRunning(!intakesRunning());
    }
    /*
     * Initializing the flywheels to off because it's not like they would ever start running anyways.
     */
    @Override
    public void init(){
        intakesRunning = false;
    }
    @Override
    public void loop(){
        addToTelemetry("Intakes running", intakesRunning);
    }
}