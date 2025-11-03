package org.firstinspires.ftc.teamcode.robot;

/*
 * This abstract class provides a blueprint for all of the subsystems to use and functions to
 * overwrite. Additionally, this lets us put all the subsystems in a list and loop through it to
 * make the code simpler and easier to change the subsystems.
 */
public abstract class Subsystem {
    protected final Robot robot;

    public Subsystem(Robot robot){
        this.robot = robot;
    }

    public void init(){}

    public void init_loop(){}

    public void start(){}

    public void loop(){}
}
