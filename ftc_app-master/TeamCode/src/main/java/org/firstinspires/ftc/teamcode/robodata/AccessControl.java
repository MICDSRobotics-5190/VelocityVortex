package org.firstinspires.ftc.teamcode.robodata;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Created by amigala on 4/20/2017.
 */

public class AccessControl {
    private boolean isG1Primary;
    private boolean isG2Primary;
    private boolean isRequesting;

    public AccessControl() {
        isG1Primary = true;
        isG2Primary = false;
        isRequesting = false;
    }

    public void changeAccess() {
        if (isG1Primary) {
            isG1Primary = false;
            isG2Primary = true;
        }

        else if (isG2Primary) {
            isG2Primary = false;
            isG1Primary = true;
        }
    }

    public boolean isG1Primary() {
        return isG1Primary;
    }

    public boolean isG2Primary() {
        return isG2Primary;
    }

    public boolean isRequesting() {
        return isRequesting;
    }

    public void setG1Primary(boolean g1Primary) {
        isG1Primary = g1Primary;
    }

    public void setRequesting(boolean requesting) {
        isRequesting = requesting;
    }

    public void setG2Primary(boolean g2Primary) {
        isG2Primary = g2Primary;
    }

    public String getTelemetryState() {
        if (isG1Primary) {
            return "P1 is Primary";
        }

        if (isG2Primary) {
            return "P2 is Primary";
        }

        return "Error with Telemetry Status for AccessControl";
    }
}