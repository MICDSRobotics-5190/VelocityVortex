package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotJackson extends OpMode{
    //Initializing the motors as DcMotors
    DcMotor grapplingPin;
    DcMotor rightMotor;
    DcMotor grapplingWinch;
    DcMotor leftMotor;
    DcMotor bucketArm;

    Servo servoMotorGateRight;
    Servo servoMotorGateLeft;
    Servo rescuerRight;
    Servo rescuerLeft;

    LightSensor sensorLight;
    LightSensor allianceDetector;

    @Override
    public void init() {
        grapplingPin = hardwareMap.dcMotor.get("grapplingPin");
        rightMotor = hardwareMap.dcMotor.get("right_drive");
        grapplingWinch = hardwareMap.dcMotor.get("grapplingWinch");
        leftMotor = hardwareMap.dcMotor.get("left_drive");
        bucketArm = hardwareMap.dcMotor.get("arm");

        servoMotorGateRight = hardwareMap.servo.get("servogr");
        servoMotorGateLeft = hardwareMap.servo.get("servogl");
        rescuerRight = hardwareMap.servo.get("rightres");
        rescuerLeft = hardwareMap.servo.get("leftres");

        sensorLight = hardwareMap.lightSensor.get("sensor1");
        allianceDetector = hardwareMap.lightSensor.get("sensor2");

        //reverses the right motor
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        float leftY = -gamepad1.left_stick_y;
        float rightY = -gamepad1.right_stick_y;

        leftMotor.setPower(leftY);
        rightMotor.setPower(rightY);

        if(gamepad1.a) {
            grapplingPin.setPower(1.0);
        }

        if(gamepad1.b){
            leftMotor.setPower(1.0);
            rightMotor.setPower(1.0);
            grapplingPin.setPower(1.0);
            grapplingWinch.setPower(1.0);
        }

        if(gamepad1.right_trigger > 0.05) {
            grapplingPin.setPower(1);
        }

        if(gamepad1.right_bumper){
            grapplingPin.setPower(-1);
        }

        if(gamepad1.left_bumper){
            grapplingWinch.setPower(-1);
        }

        if(gamepad1.left_trigger > 0.05) {
            grapplingWinch.setPower(1);
        }

        if (gamepad2.a) {
            servoMotorGateLeft.setPosition(0.75);
        }

        if (gamepad2.b) {
            servoMotorGateLeft.setPosition(.14);
        }

        if (gamepad2.x) {
            servoMotorGateRight.setPosition(0.5);
        }

        if (gamepad2.y) {
            servoMotorGateRight.setPosition(1);
        }

        if (gamepad2.left_trigger > 0.05){
            rescuerLeft.setPosition(0);
        }

        if (gamepad2.left_bumper){
            rescuerLeft.setPosition(0.5);
        }

        if (gamepad2.right_trigger > 0.05){
            rescuerRight.setPosition(1);
        }

        if (gamepad2.right_bumper){
                rescuerRight.setPosition(0.5);
        }

        if (gamepad2.left_stick_y > 0.05){
            bucketArm.setPower(1);
        }

        if (gamepad2.left_stick_y < -0.05){
            bucketArm.setPower(-0.2);
        }

        if(gamepad2.left_stick_y == 0){
            bucketArm.setPower(0);
        }

        telemetry.addData("Text", "Running!");

        //telemetry.addData("Left Motor", leftMotor.getPower());
        //telemetry.addData("Right Motor", rightMotor.getPower());
        //telemetry.addData("Grappling Hook Pin", grapplinghookPin.getPower());
        // telemetry.addData("Grappling Hook Winch", grapplingWinch.getPower());
    }
}
