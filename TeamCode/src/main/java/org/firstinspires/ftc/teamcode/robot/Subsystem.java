package org.firstinspires.ftc.teamcode.robot;

import java.util.LinkedHashMap;
import java.util.Map;

/*
 * This abstract class provides a blueprint for all of the subsystems to use and functions to
 * overwrite. Additionally, this lets us put all the subsystems in a list and loop through it to
 * make the code simpler and easier to change the subsystems.
 */
public abstract class Subsystem {
    protected final Robot robot;
    private final Map<String, Object> telemetryData = new LinkedHashMap<>();

    public Subsystem(Robot robot){
        this.robot = robot;
    }

    public void init(){}

    public void init_loop(){}

    public void start(){}

    public void loop(){}

    protected void addToTelemetry(String caption, Object value){
        telemetryData.put(caption, value);
    }
    public void clearTelemetry(){
        telemetryData.clear();
    }
    public Map<String, Object> getTelemetry(){
        return  telemetryData;
    }
}
