package org.firstinspires.ftc.teamcode;

/**
 * Created by BAbel on 1/26/2017.
 */

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Robot {

    public static DcMotor leftMotor;
    public static DcMotor rightMotor;

    public static DcMotor spinner;
    public static DcMotor flywheel;

    public static CRServo beaconSlider;


    public Robot() {
        leftMotor = null;
        rightMotor = null;

        spinner = null;
        flywheel = null;

        beaconSlider = null;
    }

    public void setupHardware(HardwareMap hardwareMap){
        leftMotor  = hardwareMap.dcMotor.get("left motor");
        rightMotor = hardwareMap.dcMotor.get("right motor");

        spinner = hardwareMap.dcMotor.get("spinner");
        flywheel = hardwareMap.dcMotor.get("flywheel");

        beaconSlider = hardwareMap.crservo.get("slider");

        // Reverse the motor that runs backwards when connected directly to the battery
        leftMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors

        beaconSlider.setDirection(CRServo.Direction.FORWARD);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        spinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void drivetrainPower(int power){
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }

    public void resetEncoders(){
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void stopMoving() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);

        spinner.setPower(0);
        flywheel.setPower(0);

        beaconSlider.setPower(0);
    }

}
