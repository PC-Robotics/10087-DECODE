package org.firstinspires.ftc.teamcode.robot.subsystem;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Subsystem;

/*
 * This class handles everything relating to the rack and pinion mechanism on the robot that raises
 * and lowers the artifacts into the flywheels.
 */
public class Elevator extends Subsystem {
    /*
     * Enum with three states, down, middle, and up so that you don't need to worry about using
     * number or anything to denote position.
     * This enum is only public so that it can be passed as an argument to setElevator.
     */
    public enum ElevatorState {
        DOWN,
        MID,
        UP,
    }
    /*
     * Declaring constants for the up, middle, and down positions of the servo.
     */
    final double ELEVATOR_UP = 0.7; // Elevator position at maximum height
    final double ELEVATOR_MID = 0.5; // Elevator position at middle height
    final double ELEVATOR_DOWN = 0.2; // Elevator position at minimum height

    /*
     * Enum to keep track of where the elevator is.
     */
    private ElevatorState elevatorState;
    /*
     * Using the parent class, Subsystem, to construct Elevator.
     */
    public Elevator(Robot robot) {
        super(robot);
    }
    /*
     * Returns true or false depending on whether the elevator is down or not. Added to have a
     * simpler alternative than robot.elevator.state() == Elevator.ElevatorState.CLOSED
     */

    public boolean isDown(){
        return elevatorState == ElevatorState.DOWN;
    }
    /*
     * Returns the state of the elevator
     */
    public ElevatorState getElevatorState(){
        return elevatorState;
    }
    /*
     * Method for setting the position of the elevator servo. Will only move the elevator if the
     * claw is set to ClawState.CLOSED, and if it's not closed, it will close the claw, unless
     * the elevator is being set to down where it doesn't matter anyways.
     *
     * The claw is quick enough to close within 1 frame, so don't worry about having to wait for
     * it to close before calling set elevator again. However, if you try and move the elevator up
     * right after you closed it, it won't be fast enough and you should wait until the next frame.
     */
    public void setElevator(ElevatorState state){
        if (robot.claw.isClosed()) {
            elevatorState = state;
            switch (state) {
                case UP:
                    robot.elevatorServo.setPosition(ELEVATOR_UP);
                    break;
                case MID:
                    robot.elevatorServo.setPosition(ELEVATOR_MID);
                    break;
                case DOWN:
                    robot.elevatorServo.setPosition(ELEVATOR_DOWN);
                    break;
            }
        } else if (state != ElevatorState.DOWN) robot.claw.setClaw(Claw.ClawState.CLOSE);
    }
    /*
     * Initializing the elevator state to down so that robot.claw.setClaw() can be called.
     */
    @Override
    public void init(){
        elevatorState = ElevatorState.DOWN;
    }
    @Override
    public void loop(){
        addToTelemetry("Elevator State", elevatorState);
    }
}