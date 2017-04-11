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

    public void setPower(double power){
        leftMotors.setPower(power);
        rightMotors.setPower(power);
    }

    public MotorPair getLeftMotors() {
        return leftMotors;
    }

    public MotorPair getRightMotors() {
        return rightMotors;
    }

    public void setModes(DcMotor.RunMode runMode){
        leftMotors.setModes(runMode);
        rightMotors.setModes(runMode);
    }

    public void resetEncoders(){
        leftMotors.setModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotors.setModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotors.setModes(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotors.setModes(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setLeftMotors(MotorPair leftMotors) {
        this.leftMotors = leftMotors;
    }

    public void setRightMotors(MotorPair rightMotors) {
        this.rightMotors = rightMotors;
    }
}
