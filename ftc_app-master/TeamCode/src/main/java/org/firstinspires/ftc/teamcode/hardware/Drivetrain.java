package org.firstinspires.ftc.teamcode.hardware;

/**
 * Created by BAbel on 4/10/2017.
 */

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Drivetrain {

    private MotorPair leftMotors;
    private MotorPair rightMotors;

    public Drivetrain(){
        leftMotors = null;
        rightMotors = null;
    }

    public Drivetrain(HardwareMap hardwareMap){
        leftMotors = new MotorPair(hardwareMap, "left front", "left back");
        rightMotors = new MotorPair(hardwareMap, "right front", "right back");
    }


}
