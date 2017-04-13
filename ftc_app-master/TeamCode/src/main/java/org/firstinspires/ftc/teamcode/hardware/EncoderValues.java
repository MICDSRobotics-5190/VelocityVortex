package org.firstinspires.ftc.teamcode.hardware;

/**
 * Created by amigala on 4/12/2017.
 */

public interface EncoderValues {
    final int GEAR_BOX_RATIO = 60; //andymark 60 motors
    final int PULSES_PER_ROTATION = 7; //PPR, from the manufacturer website
    final double GEAR_RATIO = 0.666667;

    // rotations
    final int FULL_ROTATION = (int)(GEAR_BOX_RATIO * GEAR_RATIO * PULSES_PER_ROTATION);
    final int HALF_ROTATION = FULL_ROTATION/2;
    final int THIRD_ROTATION = FULL_ROTATION/3;
    final int QUARTER_ROTATION = FULL_ROTATION/4;
}