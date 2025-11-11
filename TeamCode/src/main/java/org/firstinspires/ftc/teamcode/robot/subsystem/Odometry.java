package org.firstinspires.ftc.teamcode.robot.subsystem;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.constants.HardwareConstants;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Subsystem;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;
public class Odometry extends Subsystem {
    private Pose2D robotPosition;
    /*
     * Using the parent class, Subsystem, to construct Odometry.
     */
    public Odometry(Robot robot) {
        super(robot);
    }
    /*
     * Returns the X position of the robot.
     */
    public double getXPosition(DistanceUnit unit)
    {
        return robotPosition.getX(unit);
    }
    /*
     * Returns the Y position of the robot.
     */
    public double getYPosition(DistanceUnit unit)
    {
        return robotPosition.getY(unit);
    }
    /*
     * Returns the heading of the robot.
     */
    public double getHeading(AngleUnit unit)
    {
        return robotPosition.getHeading(unit);
    }
    public void resetYaw(){
        robot.odo.setHeading(0, AngleUnit.RADIANS);
    }
    /*
     * Method for updating robotPosition.
     */
    public void updatePosition() {
        robot.odo.update();
        robotPosition = robot.odo.getPosition();
    }
    /*
     * Method for updating the telemetry with position, velocity, status, and pinpoint frequency.
     */
    public void updateOdometryTelemetry() {
        /*
        Prints the current Position (x & y in inches, and heading in degrees) of the robot
        Can alter the DistanceUnit to display MM or INCH
        */
        String data = String.format(
                Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}",
                getXPosition(DistanceUnit.INCH),
                getYPosition(DistanceUnit.INCH),
                getHeading(AngleUnit.DEGREES)
        );
        addToTelemetry("Position", data);

        /*
         * Gets the current Velocity (x & y in in/sec and heading in degrees/sec) and prints it.
        */
        String velocity = String.format(
                Locale.US,"{XVel: %.3f, YVel: %.3f, HVel: %.3f}",
                robot.odo.getVelX(DistanceUnit.INCH),
                robot.odo.getVelY(DistanceUnit.INCH),
                robot.odo.getHeading(AngleUnit.DEGREES)
        );
        addToTelemetry("Velocity", velocity);

         /*
        Gets the Pinpoint device status. Pinpoint can reflect a few states. But we'll primarily see
        READY: the device is working as normal
        CALIBRATING: the device is calibrating and outputs are put on hold
        NOT_READY: the device is resetting from scratch. This should only happen after a power-cycle
        FAULT_NO_PODS_DETECTED - the device does not detect any pods plugged in
        FAULT_X_POD_NOT_DETECTED - The device does not detect an X pod plugged in
        FAULT_Y_POD_NOT_DETECTED - The device does not detect a Y pod plugged in
        */
        addToTelemetry("Status", robot.odo.getDeviceStatus());

        addToTelemetry("Pinpoint Frequency", robot.odo.getFrequency()); //prints/gets the current refresh rate of the Pinpoint

    }
    /*
     * Setting up offsets, encoder resolution, encoder directions, resetting pos and imu, and
     * adding some of the setup constants to telemetry.
     */
    @Override
    public void init(){
        // Setting the offsets for the odometry pods
        robot.odo.setOffsets(HardwareConstants.X_OFFSET, HardwareConstants.Y_OFFSET, DistanceUnit.MM);
        // Setting the encoder resolution for the odometry pods that we are using
        robot.odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        robot.odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        robot.odo.resetPosAndIMU();

        addToTelemetry("X offset", HardwareConstants.X_OFFSET);
        addToTelemetry("Y offset", HardwareConstants.Y_OFFSET);
        addToTelemetry("Odo device version number", robot.odo.getDeviceVersion());
        addToTelemetry("Odo device scalar", robot.odo.getYawScalar());
    }
    /*
     * Adding telemetry and updating position.
     */
    @Override
    public void loop(){
        updatePosition();
        updateOdometryTelemetry();
    }
}
