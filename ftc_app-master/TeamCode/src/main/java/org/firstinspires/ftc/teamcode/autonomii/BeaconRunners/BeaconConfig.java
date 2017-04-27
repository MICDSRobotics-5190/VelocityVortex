package org.firstinspires.ftc.teamcode.autonomii.BeaconRunners;

import org.firstinspires.ftc.teamcode.hardware.Robot;

/**
 * Created by amigala on 4/27/2017.
 */

public interface BeaconConfig {
    final double DRIVE_TRAIN_POWER = 0.5;
    final long ROTATION_TIME = 1500;
    final double LIGHT_DETECTED_THRESH = 4; // this should be in centimeters
    final long SLIDER_TIME = 1600;
    final long PULL_FORWARD_TIME = 1000;
}