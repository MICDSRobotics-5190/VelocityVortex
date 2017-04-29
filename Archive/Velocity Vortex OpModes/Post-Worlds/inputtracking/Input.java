package org.firstinspires.ftc.teamcode.inputtracking;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Created by BAbel on 4/12/2017.
 */

public class Input {

    private double currentTime;

    private double leftStickY;
    private double rightStickY;

    public Input(){

    }

    public Input(Gamepad gamepadState, double currentTime){
        leftStickY = gamepadState.left_stick_y;
        rightStickY = gamepadState.right_stick_y;
        this.currentTime = currentTime;
    }

    public double getCurrentTime() {
        return currentTime;
    }

    public double getLeftStickY() {
        return leftStickY;
    }

    public double getRightStickY() {
        return rightStickY;
    }

    public void setCurrentTime(double currentTime) {
        this.currentTime = currentTime;
    }

    public void setLeftStickY(double leftStickY) {
        this.leftStickY = leftStickY;
    }

    public void setRightStickY(double rightStickY) {
        this.rightStickY = rightStickY;
    }
}
