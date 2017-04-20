package org.firstinspires.ftc.teamcode.robodata.modestates;

/**
 * Created by amigala on 4/19/2017.
 */

public class IntakeModeState {
    private boolean isToggle;
    private boolean isContinue;

    public IntakeModeState() {
        isToggle = false;
        isContinue = true;
    }

    public IntakeModeState(String mode) {
        switch (mode) {
            case "toggle" :
                isToggle = true;
                isContinue = false;
                break;
            case "continue" :
                isToggle = false;
                isContinue = true;
                break;
        }
    }

    public void changeMode() {
        if (isToggle) {
            isToggle = false;
            isContinue = true;
        }
        else if (isContinue) {
            isContinue = false;
            isToggle = true;
        }
    }

    public String getTelemetryStatus() {
        if (isToggle) {
            return "Alex Mode";
        }

        else if (isContinue) {
            return "Blake Mode";
        }

        return "Mode Can't Be Determined!";
    }

    public void setContinue(boolean aContinue) {
        isContinue = aContinue;
    }

    public void setToggle(boolean toggle) {
        isToggle = toggle;
    }

    public boolean isContinue() {
        return isContinue;
    }

    public boolean isToggle() {
        return isToggle;
    }
}