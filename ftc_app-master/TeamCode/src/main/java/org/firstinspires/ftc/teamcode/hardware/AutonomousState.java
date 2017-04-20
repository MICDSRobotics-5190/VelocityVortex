package org.firstinspires.ftc.teamcode.hardware;

/**
 * Created by amigala on 4/20/2017.
 */

public class AutonomousState {
    private int step;

    public AutonomousState() {
        step = 0;
    }

    public AutonomousState(int s) {
        step = s;
    }

    public void incrementState() {
        step++;
    }

    public void decrementState() {
        step--;
    }

    public void setState(int s) {
        step = s;
    }

    public int getState() {
        return step;
    }

    public String getTelemetryState() {
        return String.valueOf(step);
    }
}