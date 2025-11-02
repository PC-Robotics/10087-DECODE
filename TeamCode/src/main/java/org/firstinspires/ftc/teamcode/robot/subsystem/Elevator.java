package org.firstinspires.ftc.teamcode.robot.subsystem;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Subsystem;

public class Elevator extends Subsystem {
    final double ELEVATOR_UP = 0.7; // Elevator position at maximum height
    final double ELEVATOR_MID = 0.5; // Elevator position at middle height
    final double ELEVATOR_DOWN = 0.2; // Elevator position at minimum height
    private ElevatorState elevatorState;

    public Elevator(Robot robot) {
        super(robot);
    }
    public enum ElevatorState {
        DOWN,
        MID,
        UP,
    }

    public boolean isDown(){
        return elevatorState == ElevatorState.DOWN;
    }
    public ElevatorState state(){
        return elevatorState;
    }
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

    @Override
    public void init(){
        elevatorState = ElevatorState.DOWN;
    }
}