package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

public class BabbyBot extends OpMode {

    DcMotor rightDrive;
    DcMotor leftDrive;
    DcMotor roller;
    DcMotor winch;

    //int numOpLoops = 1;
    boolean driveToggle = true;

    @Override
    public void init() {
        leftDrive = hardwareMap.dcMotor.get("left_drive");
        rightDrive = hardwareMap.dcMotor.get("right_drive");
        roller = hardwareMap.dcMotor.get("roller");
        winch = hardwareMap.dcMotor.get("winch");

        //reverses the left motor
        rightDrive.setDirection(DcMotor.Direction.REVERSE);


    }

    @Override
    public void loop() {


        rightDrive.setPower(-gamepad1.right_stick_y);
        leftDrive.setPower(-gamepad1.left_stick_y);

        if(gamepad1.right_trigger > 0.2){
            roller.setPower(1);
        } else if (gamepad1.left_trigger > 0.2){
            roller.setPower(-1);
        } else {
            roller.setPower(0);
        }

        if(gamepad1.right_bumper){
            winch.setPower(-1);
        } else if (gamepad1.left_bumper){
            winch.setPower(1);
        } else {
            winch.setPower(0);
        }

        telemetry.addData("Text", "Running!");

        /*
        //Setting the motor controller to be able to be read as much as possible (takes time to switch)
        if(numOpLoops % 17 == 0){
            driveController.setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_ONLY);
        }

        //Reading, and switching back after.
        if(driveController.getMotorControllerDeviceMode() == DcMotorController.DeviceMode.READ_ONLY){
            telemetry.addData("Right Motor", rightDrive.getPower());
            telemetry.addData("Left Motor", leftDrive.getPower());
            driveController.setMotorControllerDeviceMode(DcMotorController.DeviceMode.WRITE_ONLY);
            numOpLoops = 0;
        }

        numOpLoops++;
        */

    }
}   //TO-DO: Write stuff for everything else the robot might have on it.
// Correct the motors being weird af on autonomous and such from the quick fix left/right
