package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

public class PopsicleDrive extends OpMode {

    DcMotor rightDrive;
    DcMotor leftDrive;
    DcMotorController driveController;
    DcMotor arm;
    DcMotor sweeper;
    DcMotor armConveyor;
    DcMotor rope;

    Servo rightPoleGrab;
    Servo leftPoleGrab;

    LightSensor lineSensor;
    ColorSensor allianceSensor;

    //int numOpLoops = 1;
    boolean driveToggle = true;

    @Override
    public void init() {
        leftDrive = hardwareMap.dcMotor.get("right_drive");
        rightDrive = hardwareMap.dcMotor.get("left_drive");
        driveController = hardwareMap.dcMotorController.get("drive");
        arm = hardwareMap.dcMotor.get("arm");
        sweeper = hardwareMap.dcMotor.get("sweeper");
        armConveyor = hardwareMap.dcMotor.get("conveyor");
        rope = hardwareMap.dcMotor.get("rope");

        rightPoleGrab = hardwareMap.servo.get("right_res");
        leftPoleGrab = hardwareMap.servo.get("left_res");

        lineSensor = hardwareMap.lightSensor.get("line_sensor");
        allianceSensor = hardwareMap.colorSensor.get("ally_sensor");

        driveController.setMotorControllerDeviceMode(DcMotorController.DeviceMode.WRITE_ONLY);

        //reverses the left motor
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        rightPoleGrab.setPosition(0);
        leftPoleGrab.setPosition(0.9);
    }

    @Override
    public void loop() {

        float leftY = -gamepad1.left_stick_y;
        float rightY = -gamepad1.right_stick_y;

        //rightDrive.setPower(rightY);
        //leftDrive.setPower(leftY);

        if(driveToggle) {
            rightDrive.setPower(rightY);
            leftDrive.setPower(leftY);
        } else {
            rightDrive.setPower(-rightY);
            leftDrive.setPower(-leftY);
        }


        if(gamepad1.a){
            driveToggle = true;
        }

        if(gamepad1.b){
            driveToggle = false;
        }

        float sexyStan = -gamepad2.right_stick_y;

        arm.setPower(sexyStan);

        if(gamepad2.back || gamepad2.left_stick_button || gamepad2.right_stick_button || gamepad2.left_trigger > 0){
            rope.setPower(-0.35);
        } else if (gamepad2.start || gamepad2.right_trigger > 0){
            rope.setPower(0.35);
        } else {
            rope.setPower(0);
        }

        if(gamepad2.left_bumper){
            sweeper.setPower(-1);
        } else if(gamepad2.right_bumper){
            sweeper.setPower(1);
        } else {
            sweeper.setPower(0);
        }


        if(gamepad1.left_bumper || gamepad1.dpad_left){
            armConveyor.setPower(-1);
        } else if(gamepad1.right_bumper || gamepad1.dpad_right){
            armConveyor.setPower(1);
        } else {
            armConveyor.setPower(0);
        }


        //Servo positions need to be re-found. Could use telemetry and slowly adding a position up and down like OG code

        /*
        if(gamepad1.x){
            if(rightPoleGrab.getPosition() == 0 || leftPoleGrab.getPosition() == 0){
                telemetry.addData("Right Servo", "STOP - Nicole Truman-Shaw");
                telemetry.addData("Left Servo", "STOP - Nicole Truman-Shaw");
            } else {
                rightPoleGrab.setPosition(rightPoleGrab.getPosition() - 0.01);
                leftPoleGrab.setPosition(leftPoleGrab.getPosition() - 0.01);
            }
        }


        if (gamepad1.y) {
            if(rightPoleGrab.getPosition() == 1 || leftPoleGrab.getPosition() == 1){
                telemetry.addData("Right Servo", "STOP - Nicole Truman-Shaw");
                telemetry.addData("Left Servo", "STOP - Nicole Truman-Shaw");
            } else {
                rightPoleGrab.setPosition(rightPoleGrab.getPosition() + 0.01);
                leftPoleGrab.setPosition(leftPoleGrab.getPosition() + 0.01);
            }
        }
        */

        //Up
        if(gamepad1.y){
            rightPoleGrab.setPosition(0);
            leftPoleGrab.setPosition(0.9);
        }

        //Down
        if(gamepad1.x){
            rightPoleGrab.setPosition(0.4);
            leftPoleGrab.setPosition(0.5);
        }

        telemetry.addData("Text", "Running!");
        telemetry.addData("Right Servo", rightPoleGrab.getPosition());
        telemetry.addData("Left Servo", leftPoleGrab.getPosition());

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
