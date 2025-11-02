package org.firstinspires.ftc.teamcode.robot;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robot.subsystem.Claw;
import org.firstinspires.ftc.teamcode.robot.subsystem.DrivetrainFieldCentric;
import org.firstinspires.ftc.teamcode.robot.subsystem.Elevator;
import org.firstinspires.ftc.teamcode.robot.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.subsystem.Flywheels;

import java.util.ArrayList;
import java.util.List;

public class Robot {
    public Claw claw;
    public Elevator elevator;
    public Drivetrain drivetrain;
    public DrivetrainFieldCentric drivetrainFieldCentric;
    public Flywheels flywheels;
    private final List<Subsystem> subsystems = new ArrayList<>();

    final double FEED_TIME_SECONDS = 1; // The amount of time we wait before lowering the elevator and stopping the flywheels.

    // Declare OpMode members.
    public DcMotor leftFrontDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor leftBackDrive = null;
    public DcMotor rightBackDrive = null;
    public DcMotorEx leftFlywheel = null;
    public DcMotorEx rightFlywheel = null;
    public Servo elevatorServo = null;
    public Servo clawServo = null;
    public IMU imu = null;
    private IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
            RevHubOrientationOnRobot.UsbFacingDirection.UP
    ));

    public void init(HardwareMap hardwareMap) {
        /*
         * Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step.
         */
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        leftFlywheel = hardwareMap.get(DcMotorEx.class, "left_launcher");
        rightFlywheel = hardwareMap.get(DcMotorEx.class, "right_launcher");
        elevatorServo = hardwareMap.get(Servo.class, "elevator");
        clawServo = hardwareMap.get(Servo.class, "claw");
        imu = hardwareMap.get(IMU.class, "imu");

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
        leftFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to
         * slow down much faster when it is coasting. This creates a much more controllable
         * drivetrain. As the robot stops much quicker.
         */
        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        leftFlywheel.setZeroPowerBehavior(BRAKE);
        rightFlywheel.setZeroPowerBehavior(BRAKE);

        leftFlywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
        rightFlywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

        leftFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        imu.initialize(parameters);

        claw = new Claw(this);
        elevator = new Elevator(this);
        drivetrain = new Drivetrain(this);
        drivetrainFieldCentric = new DrivetrainFieldCentric(this);
        flywheels = new Flywheels(this);

        subsystems.add(claw);
        subsystems.add(elevator);
        subsystems.add(drivetrain);
        subsystems.add(drivetrainFieldCentric);
        subsystems.add(flywheels);

        for (Subsystem s: subsystems){
            s.init();
        }
    }

    public void init_loop(){
        for (Subsystem s: subsystems){
            s.init_loop();
        }
    }

    public void start(){
        for (Subsystem s: subsystems){
            s.start();
        }
    }

    public void loop(){
        for (Subsystem s: subsystems){
            s.loop();
        }
    }
}
