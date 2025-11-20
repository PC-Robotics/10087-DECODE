package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.HardwareConstants;
import org.firstinspires.ftc.teamcode.robot.subsystem.Claw;
import org.firstinspires.ftc.teamcode.robot.subsystem.DrivetrainPID;
import org.firstinspires.ftc.teamcode.robot.subsystem.Elevator;
import org.firstinspires.ftc.teamcode.robot.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.subsystem.Flywheels;
import org.firstinspires.ftc.teamcode.robot.subsystem.Intakes;
import org.firstinspires.ftc.teamcode.robot.subsystem.Odometry;

import java.util.ArrayList;
//import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
/*
 * This class handles the setup for all the different subsystems of the robot as well as providing
 * a space for any extra methods, such as launch() to be.
 */

public class Robot extends RobotSetup {
    /*
     * Declaring all of the subsystems as well as the subsystem list.
     */
    public Claw claw;
    public Elevator elevator;
    public DrivetrainPID drivetrain;
    public Flywheels flywheels;
    public Intakes intakes;
    public Odometry odometry;
    private final List<Subsystem> subsystems = new ArrayList<>();

    /*
     * Declaring constants and enums to be used in robot methods.
     */
    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
        LOWERING,
    }

    private LaunchState launchState; // Enum to keep track of what stage of launching we are in
    ElapsedTime feederTimer = new ElapsedTime(); // Timer to be used when launching artifacts

    /*
     * Intializing RobotSetup as well as assigning all of the the components, adding them to the
     * subsystem list, and then initializing everything in the subsystem list.
     */
    @Override
    public void init(HardwareMap hardwareMap) {
        super.init(hardwareMap);
        claw = new Claw(this);
        elevator = new Elevator(this);
        drivetrain = new DrivetrainPID(this);
        flywheels = new Flywheels(this);
        intakes = new Intakes(this);
        odometry = new Odometry(this);

        subsystems.add(odometry);
        subsystems.add(claw);
        subsystems.add(elevator);
        subsystems.add(drivetrain);
        subsystems.add(flywheels);
        subsystems.add(intakes);

        for (Subsystem s: subsystems){
            s.init();
        }

        launchState = LaunchState.IDLE;
    }

    /*
     * Code to be ran every frame after initialization but before start.
     */
    public void init_loop(){
        for (Subsystem s: subsystems){
            s.init_loop();
        }
    }

    /*
     * Code to be ran once on start.
     */
    public void start(){
        for (Subsystem s: subsystems){
            s.clearTelemetry();
            s.start();
        }
    }

    /*
     * Code to be ran every frame after start.
     */
    public void loop(){
        for (Subsystem s: subsystems){
            s.loop();
        }
    }

    /*
     * Iterating through the list of subsystems to get their telemetry maps and then iterating
     * through each subsystems telemetry map, getting the captions and the values of their telemetry,
     * and adding it to the Teleop's telemetry.
     */
    public void addTelemetry(Telemetry telemetry){
        for (Subsystem s: subsystems){
            Map<String, Object> telemetryData = s.getTelemetry();

            for (Map.Entry<String, Object> entry : telemetryData.entrySet()){
                String caption = entry.getKey();
                Object value = entry.getValue();

                telemetry.addData(caption, value);
            }
        }

        telemetry.addData("Launch state", launchState);
    }

    public void launch(boolean shotRequested) {
        switch (launchState) {
            case IDLE:
                if (shotRequested) { // Setting the launch state to spin up when the shoot button is pressed
                    launchState = LaunchState.SPIN_UP;
                    elevator.setElevator(Elevator.ElevatorState.MID);
                }
                break;
            case SPIN_UP:
                flywheels.setFlywheels(true);
                if (flywheels.flywheelsReady()) {
                    launchState = LaunchState.LAUNCH; // Launching once the motor reaches the right speed
                }
                break;
            case LAUNCH:
                elevator.setElevator(Elevator.ElevatorState.UP);
                feederTimer.reset(); // Starting timer for the elevator
                launchState = LaunchState.LAUNCHING;
                break;

            case LAUNCHING:
                /*
                 * Lowering elevator, stopping flywheels, and going back to idle once the timer elapses.
                 */
                if (feederTimer.seconds() > HardwareConstants.FEED_TIME_SECONDS) {
                    elevator.setElevator(Elevator.ElevatorState.DOWN);
                    feederTimer.reset();
                    flywheels.setFlywheels(false);
                    launchState = LaunchState.LOWERING;
                }
                break;

            case LOWERING:
                if (feederTimer.seconds() > HardwareConstants.LOWER_TIME_SECONDS){
                    claw.setClaw(Claw.ClawState.OPEN);
                    launchState = LaunchState.IDLE;
                }
                break;
        }
    }
}
