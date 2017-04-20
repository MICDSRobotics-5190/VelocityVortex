package org.firstinspires.ftc.teamcode.robodata;

/**
 * Created by amigala on 4/12/2017.
 */

public interface EncoderValues {
    final int GEAR_BOX_RATIO = 60; //andymark 60 motors
    final int PULSES_PER_REVOLUTION = 7; //PPR, from the manufacturer website
    final int COUNTS_PER_REVOLUTION = 1680;
    final double LAUNCHER_GEAR_RATIO = 0.66666666667;
    final double REAL_WORLD_CONSTANT = 0.9;

    // Launcher rotations
    final int FULL_ROTATION = (int)(LAUNCHER_GEAR_RATIO * COUNTS_PER_REVOLUTION * REAL_WORLD_CONSTANT);


}