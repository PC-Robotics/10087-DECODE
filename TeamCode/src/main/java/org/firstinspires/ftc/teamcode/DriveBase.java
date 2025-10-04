/*
 * Copyright (c) 2025 FIRST
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior
 * written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This file includes a teleop (driver-controlled) file for the goBILDA® StarterBot for the
 * 2025-2026 FIRST® Tech Challenge season DECODE™. It leverages a differential/Skid-Steer
 * system for robot mobility, one high-speed motor driving two "launcher wheels", and two servos
 * which feed that launcher.
 *
 * Likely the most niche concept we'll use in this example is closed-loop motor velocity control.
 * This control method reads the current speed as reported by the motor's encoder and applies a varying
 * amount of power to reach, and then hold a target velocity. The FTC SDK calls this control method
 * "RUN_USING_ENCODER". This contrasts to the default "RUN_WITHOUT_ENCODER" where you control the power
 * applied to the motor directly.
 * Since the dynamics of a launcher wheel system varies greatly from those of most other FTC mechanisms,
 * we will also need to adjust the "PIDF" coefficients with some that are a better fit for our application.
 */

@TeleOp(name = "Standard Teleop", group = "Teleop")
//@Disabled
public class DriveBase extends OpMode {
    final double STOP_SPEED = 0.0; // We send this power to the servos/motors when we want them to stop.
    final double FULL_SPEED = 1.0;

    final double FEED_TIME_SECONDS = 4; // The amount of time we wait before lowering the elevator and stopping the flywheels.
    final double ELEVATOR_UP = 1.0; // Elevator position at maximum height
    final double ELEVATOR_DOWN = 0.5; // Elevator position at minimum height

    final double CLAW_OPEN = 0.45; // Claw position closed
    final double CLAW_CLOSE = 0.5; // Claw position open

    /*
     * When we control our launcher motor, we are using encoders. These allow the control system
     * to read the current speed of the motor and apply more or less power to keep it at a constant
     * velocity. Here we are setting the target, and minimum velocity that the launcher should run
     * at. The minimum velocity is a threshold for determining when to fire.
     */

    // Declare OpMode members.
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor leftLauncher = null;
    private DcMotor rightLauncher = null;
    private Servo elevator = null;
    private Servo claw = null;

    ElapsedTime feederTimer = new ElapsedTime();

    /*
     * TECH TIP: State Machines
     * We use a "state machine" to control our launcher motor and feeder servos in this program.
     * The first step of a state machine is creating an enum that captures the different "states"
     * that our code can be in.
     * The core advantage of a state machine is that it allows us to continue to loop through all
     * of our code while only running specific code when it's necessary. We can continuously check
     * what "State" our machine is in, run the associated code, and when we are done with that step
     * move on to the next state.
     * This enum is called the "LaunchState". It reflects the current condition of the shooter
     * motor and we move through the enum when the user asks our code to fire a shot.
     * It starts at idle, when the user requests a launch, we enter SPIN_UP where we get the
     * motor up to speed, once it meets a minimum speed then it starts and then ends the launch process.
     * We can use higher level code to cycle through these states. But this allows us to write
     * functions and autonomous routines in a way that avoids loops within loops, and "waits".
     */
    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }

    private LaunchState launchState;

    // Setup a variable for each drive wheel to save power level for telemetry
    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        launchState = LaunchState.IDLE;

        /*
         * Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step.
         */
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        leftLauncher = hardwareMap.get(DcMotor.class, "left_launcher");
        rightLauncher = hardwareMap.get(DcMotor.class, "right_launcher");
        elevator = hardwareMap.get(Servo.class, "elevator");
        claw = hardwareMap.get(Servo.class, "claw");

        /*
         * To drive forward, most robots need the motor on one side to be reversed,
         * because the axles point in opposite directions. Pushing the left stick forward
         * MUST make robot go forward. So adjust these two lines based on your first test drive.
         *
         * Front wheels have been inverted to account for the 90 degree bends.
         */
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        /*
         * Here we set our launcher to the RUN_USING_ENCODER runmode.
         * If you notice that you have no control over the velocity of the motor, it just jumps
         * right to a number much higher than your set point, make sure that your encoders are plugged
         * into the port right beside the motor itself. And that the motors polarity is consistent
         * through any wiring.
         */
        leftLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLauncher.setDirection(DcMotorSimple.Direction.REVERSE);

        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to
         * slow down much faster when it is coasting. This creates a much more controllable
         * drivetrain. As the robot stops much quicker.
         */
        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        leftLauncher.setZeroPowerBehavior(BRAKE);
        rightLauncher.setZeroPowerBehavior(BRAKE);

        leftLauncher.setZeroPowerBehavior(BRAKE);
        rightLauncher.setZeroPowerBehavior(BRAKE);

        /*
         * Inverting the direction of the elevator servo to account for the rack and pinion.
         */
        elevator.setDirection(Servo.Direction.FORWARD);
        claw.setDirection(Servo.Direction.REVERSE);

        /*
         * Tell the driver that initialization is complete.
         */
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        /*
         * Here we call a function called arcadeDrive. The arcadeDrive function takes the input from
         * the joysticks, and applies power to the left and right drive motor to move the robot
         * as requested by the driver. "arcade" refers to the control style we're using here.
         * Much like a classic arcade game, when you move the left joystick forward both motors
         * work to drive the robot forward, and when you move the right joystick left and right
         * both motors work to rotate the robot. Combinations of these inputs can be used to create
         * more complex maneuvers.
         */
        mecanumDrive(gamepad1.left_stick_y, -gamepad1.right_stick_x, gamepad1.left_stick_x);

        /*
         * Here we give the user control of the speed of the launcher motor without automatically
         * queuing a shot.
         */
        if (gamepad1.triangle) {
            leftLauncher.setPower(FULL_SPEED);
            rightLauncher.setPower(FULL_SPEED);
        } else if (gamepad1.square) { // stop flywheels
            leftLauncher.setPower(STOP_SPEED);
            rightLauncher.setPower(STOP_SPEED);
        }

        /*
         * Opening and closing the claw.
         */
        if (gamepad1.circle) setClaw(true);// open claw
        else if (gamepad1.cross) setClaw(false); // close claw

        if (gamepad1.dpad_up) raiseElevator(true);// raise elevator
        else if (gamepad1.dpad_down) raiseElevator(false); // lower elevator


        /*
         * Now we call our "Launch" function.
         */
        launch(gamepad1.rightBumperWasPressed());

        /*
         * Show the state, motor powers, and servo positions.
         */
        telemetry.addData("State", launchState);
        telemetry.addData("Elevator position", elevator.getPosition());
        telemetry.addData("Claw position", claw.getPosition());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    void mecanumDrive(double forward, double strafe, double rotate){

        /* the denominator is the largest motor power (absolute value) or 1
         * This ensures all the powers maintain the same ratio,
         * but only if at least one is out of the range [-1, 1]
         */
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);

        leftFrontPower = (forward + strafe + rotate) / denominator;
        rightFrontPower = (forward - strafe - rotate) / denominator;
        leftBackPower = (forward - strafe + rotate) / denominator;
        rightBackPower = (forward + strafe - rotate) / denominator;

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

    }

    void launch(boolean shotRequested) {
        switch (launchState) {
            case IDLE:
                if (shotRequested) { // Setting the launch state to spin up when the shoot button is pressed
                    launchState = LaunchState.SPIN_UP;
                    feederTimer.reset();
                }
                break;
            case SPIN_UP:
                leftLauncher.setPower(FULL_SPEED);
                rightLauncher.setPower(FULL_SPEED);
                if (feederTimer.seconds()>1.0) { // Launching once the motor reaches the right speed
                    launchState = LaunchState.LAUNCH;
                }
                break;
            case LAUNCH:
                if (claw.getPosition()>CLAW_CLOSE) { // launching if claw is closed
                    elevator.setPosition((ELEVATOR_UP));
                    feederTimer.reset(); // Starting timer for the elevator
                    launchState = LaunchState.LAUNCHING;
                } else {
                    setClaw(false); // closing claw
                }
                break;
            case LAUNCHING:
                /*
                 * Lowering elevator, stopping flywheels, and going back to idle once the timer elapses.
                 */
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    launchState = LaunchState.IDLE;
                    elevator.setPosition((ELEVATOR_DOWN));
                    leftLauncher.setPower(STOP_SPEED);
                    rightLauncher.setPower(STOP_SPEED);
                }
                break;
        }
    }

    /*
     * Open claw. Close claw.
     * If anyone says that I need to comment this function better, you should retake CSP.
     */
    void setClaw(boolean open){
        if(open){
            claw.setPosition(CLAW_OPEN);
        } else {
            claw.setPosition(CLAW_CLOSE);
        }
    }

    void raiseElevator(boolean up){
        if(up){
            elevator.setPosition(ELEVATOR_UP);
        } else {
            elevator.setPosition(ELEVATOR_DOWN);
        }
    }
}