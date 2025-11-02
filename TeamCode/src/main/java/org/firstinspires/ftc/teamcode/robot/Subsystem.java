package org.firstinspires.ftc.teamcode.robot;

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
