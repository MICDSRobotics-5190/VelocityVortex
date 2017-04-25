package org.firstinspires.ftc.teamcode.robodata;

/**
 * Created by amigala on 4/12/2017.
 */

public interface EncoderValues {

    int GEAR_BOX_RATIO = 60; //andymark 60 motors
    int PULSES_PER_REVOLUTION = 7; //PPR, from the manufacturer website

    int COUNTS_PER_REVOLUTION = 1680;
    double LAUNCHER_GEAR_RATIO = 0.66666666667;
    double REAL_WORLD_CONSTANT = 0.9;
    double DRIVETRAIN_GEAR_RATIO = 0.33333333333;

    int FLOOR_TILE = (int)(2250 * DRIVETRAIN_GEAR_RATIO);
    int QUARTER_TURN = 1300;
    int FULL_TURN = QUARTER_TURN * 4;

    int MAX_SPEED = (int)((GEAR_BOX_RATIO * PULSES_PER_REVOLUTION) / DRIVETRAIN_GEAR_RATIO);

    // Launcher rotations
    final int FULL_ROTATION = (int)(LAUNCHER_GEAR_RATIO * COUNTS_PER_REVOLUTION * REAL_WORLD_CONSTANT);


}