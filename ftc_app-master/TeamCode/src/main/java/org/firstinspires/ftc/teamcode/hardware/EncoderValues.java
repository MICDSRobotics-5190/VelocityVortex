package org.firstinspires.ftc.teamcode.hardware;

/**
 * Created by amigala on 4/12/2017.
 */

public interface EncoderValues {
    int GEAR_BOX_RATIO = 60; //andymark 60 motors
    int PULSES_PER_ROTATION = 7; //PPR, from the manufacturer website
    double GEAR_RATIO = 0.666667;

    // rotations
    int FULL_ROTATION = (int)(GEAR_BOX_RATIO * GEAR_RATIO * PULSES_PER_ROTATION);
    int HALF_ROTATION = FULL_ROTATION/2;
    int THIRD_ROTATION = FULL_ROTATION/3;
    int QUARTER_ROTATION = FULL_ROTATION/4;
}