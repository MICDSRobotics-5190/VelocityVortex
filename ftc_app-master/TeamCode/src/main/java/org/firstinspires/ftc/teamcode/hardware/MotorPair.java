package org.firstinspires.ftc.teamcode.hardware;

/**
 * Created by BAbel on 4/10/2017.
 */

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MotorPair implements Drivetrain{

    private DcMotor motor1;
    private DcMotor motor2;

    public MotorPair(){
        motor1 = null;
        motor2 = null;
    }

    public MotorPair(HardwareMap hardwareMap, String hardwareName1, String hardwareName2){
        motor1 = hardwareMap.dcMotor.get(hardwareName1);
        motor2 = hardwareMap.dcMotor.get(hardwareName2);
    }

    @Override
    public void setPower(double power){
        motor1.setPower(power);
        motor2.setPower(power);
    }

    @Override
    public void stopMoving(){
        motor1.setPower(0);
        motor2.setPower(0);
    }

    public void setFrontPower(double power){
        motor1.setPower(power);
    }
    public void setBackPower(double power){
        motor2.setPower(power);
    }

    public void getFrontPower(double power) { motor1.getPower(); }
    public void getBackPower(double power) { motor2.getPower(); }

    public void setDirections(DcMotorSimple.Direction direction){
        motor1.setDirection(direction);
        motor2.setDirection(direction);
    }

    @Override
    public void setModes(DcMotor.RunMode runMode){
        motor1.setMode(runMode);
        motor2.setMode(runMode);
    }

    @Override
    public void resetEncoders(){
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public DcMotor getMotor1() {
        return motor1;
    }

    public DcMotor getMotor2() {
        return motor2;
    }

    public void setMotor1(DcMotor motor1) {
        this.motor1 = motor1;
    }

    public void setMotor2(DcMotor motor2) {
        this.motor2 = motor2;
    }
}
