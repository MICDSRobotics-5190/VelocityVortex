package org.firstinspires.ftc.teamcode.hardware;

/**
 * Created by BAbel on 4/10/2017.
 */

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


public interface Drivetrain {

    public void setPower(double power);

    public void setModes(DcMotor.RunMode runMode);

    public void resetEncoders();

    public void stopMoving();

}
