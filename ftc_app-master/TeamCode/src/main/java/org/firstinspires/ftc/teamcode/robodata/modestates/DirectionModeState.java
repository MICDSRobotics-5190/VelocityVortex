package org.firstinspires.ftc.teamcode.robodata.modestates;

/**
 * Created by amigala on 4/20/2017.
 */

public class DirectionModeState {
    private boolean isForward;
    private double multiplier;

    public DirectionModeState() {
        isForward = true;
        multiplier = 1;
    }

    public void shiftGears() {
        if (isForward) {
            isForward = false;
            multiplier = 1;
        }
        else {
            isForward = true;
            multiplier = -1;
        }
    }

    public String getTelemetryMessage() {
        if (!isForward) {
            return "R";
        }
        else {
            return "D";
        }
    }

    public double getDirection() {
        return multiplier;
    }
}