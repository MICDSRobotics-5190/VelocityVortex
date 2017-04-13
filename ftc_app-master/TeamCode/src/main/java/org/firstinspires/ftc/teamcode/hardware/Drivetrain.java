package org.firstinspires.ftc.teamcode.hardware;

/**
 * Created by BAbel on 4/10/2017.
 */

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


public interface Drivetrain {

    int GEAR_BOX_RATIO = 60; //andymark 60 motors
    int PULSES_PER_ROTATION = 7; //PPR, from the manufacturer website

    public void setPower(double power);

    public void setModes(DcMotor.RunMode runMode);

    public void resetEncoders();

    public void stopMoving();

}
