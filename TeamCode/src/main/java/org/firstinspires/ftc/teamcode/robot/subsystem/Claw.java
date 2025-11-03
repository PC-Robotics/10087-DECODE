package org.firstinspires.ftc.teamcode.robot.subsystem;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Subsystem;

/*
 * This class handles everything relating to the moving claw on the robot that opens and closes
 * using a servo.
 */
public class Claw extends Subsystem {
    /*
     * Enum with two states, open and close, so that there is no confusion with a boolean value that
     * could be true when the claw is open or true when the claw is closed.
     * This enum is only public so that it can be passed as an argument to setClaw in other classes.
     */
    public enum ClawState {
        CLOSE,
        OPEN,
    }
    /*
     * Declaring constants for the open and closed positions of the servo.
     */
    final double CLAW_OPEN = 0.20; // Claw position closed
    final double CLAW_CLOSE = 0.3; // Claw position open

    /*
     * Enum to keep track of whether or not the claw is open or not.
     */
    private ClawState clawState;
    /*
     * Using the parent class, Subsystem, to construct Claw.
     */
    public Claw(Robot robot) {
        super(robot);
    }
    /*
     * Returns true or false depending on whether the claw is down or not. Added to have a simpler
     * alternative than robot.claw.state() == Claw.ClawState.CLOSED
     */
    public boolean isClosed(){
        return clawState == ClawState.CLOSE;
    }
    /*
     * Returns the state of the claw
     */
    public ClawState state(){
        return clawState;
    }
    /*
     * Method for setting the position of the claw servo. Will only open the claw if the elevator
     * is set to ElevatorState.DOWN, but with present robot configuration, it can not actually
     * determine the real location of the elevator, meaning that if the elevator is in the process
     * of lowering and setClaw(ClawState.OPEN) is called, the claw will open.
     *
     * If the elevator is not set to down, the code does nothing.
     */
    public void setClaw(ClawState state){
        switch (state) {
            case OPEN:
                if (robot.elevator.isDown()) {
                    clawState = state;
                    robot.clawServo.setPosition(CLAW_OPEN);
                }
                break;
            case CLOSE:
                clawState = state;
                robot.clawServo.setPosition(CLAW_CLOSE);
                break;
        }
    }
    /*
     * Toggles the claw between open and close. It does not wait until after the claw moves before
     * it can be called again, so if this is being called in a teleop, don't use gamepad.button to
     * call this, because the button will be pressed for multiple frames. Instead, opt for
     * gamepad.buttonWasPressed() to differentiate between separate button presses.
     */
    public void toggleClaw(){
        switch (clawState) {
            case OPEN:
                setClaw(ClawState.CLOSE);
                break;
            case CLOSE:
                setClaw(ClawState.OPEN);
                break;
        }
    }
    /*
     * Initializing the claw state to closed so that robot.elevator.setElevator can be called.
     */
    @Override
    public void init(){
        clawState = ClawState.CLOSE;
    }
}