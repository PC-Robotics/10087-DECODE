package org.firstinspires.ftc.teamcode.robot.subsystem;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Subsystem;

public class Claw extends Subsystem {
    final double CLAW_OPEN = 0.20; // Claw position closed
    final double CLAW_CLOSE = 0.3; // Claw position open
    private ClawState clawState;

    public Claw(Robot robot) {
        super(robot);
    }
    public enum ClawState {
        CLOSE,
        OPEN,
    }

    public boolean isClosed(){
        return clawState == ClawState.CLOSE;
    }
    public ClawState state(){
        return clawState;
    }
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
}