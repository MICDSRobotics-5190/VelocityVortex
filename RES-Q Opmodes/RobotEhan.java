package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by BAbel on 9/21/2015.
 */
public class RobotEhan extends OpMode{
    //Initializing the motors as DcMotors
    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor tubeArm;
    DcMotor ballLauncherLeft;
    DcMotor ballLauncherRight;


    @Override
    public void init() {
        leftMotor = hardwareMap.dcMotor.get("left_drive");
        rightMotor = hardwareMap.dcMotor.get("right_drive");
        tubeArm = hardwareMap.dcMotor.get("tubeArm");
        ballLauncherLeft = hardwareMap.dcMotor.get("ballLauncherLeft");
        ballLauncherRight = hardwareMap.dcMotor.get("ballLauncherRight");


        //reverses the right motor
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        float leftY = -gamepad1.left_stick_y;
        float rightY = -gamepad1.right_stick_y;

        leftMotor.setPower(leftY);
        rightMotor.setPower(rightY);

        if(gamepad1.dpad_up) {
            tubeArm.setPower(0.5);
        }

        if(gamepad1.dpad_down) {
            tubeArm.setPower(-0.5);
        }

        if(gamepad1.a) {
            ballLauncherLeft.setPower(1.0);
            ballLauncherRight.setPower(1.0);
        }

        //Legacy motors are write-only

        //telemetry.addData("Left Motor", leftMotor.getPower());
        //telemetry.addData("Right Motor", rightMotor.getPower());
        //telemetry.addData("Tube Motor", tubeArm.getPower());
        //telemetry.addData("Baseball Launchers", ballLauncherLeft.getPower());

    }
}
