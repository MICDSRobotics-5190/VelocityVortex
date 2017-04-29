package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotClayton extends OpMode{
    //Initializing the motors as DcMotors
    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor bucketMotor;
    Servo servoMotor;

    @Override
    public void init() {
        leftMotor = hardwareMap.dcMotor.get("left_drive");
        rightMotor = hardwareMap.dcMotor.get("right_drive");
        bucketMotor = hardwareMap.dcMotor.get("buckets");
        servoMotor = hardwareMap.servo.get("servo");

        //reverses the right motor
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        float leftY = -gamepad1.left_stick_y;
        float rightY = -gamepad1.right_stick_y;

        leftMotor.setPower(leftY);
        rightMotor.setPower(rightY);

        if (gamepad1.right_bumper) {
            servoMotor.setPosition(servoMotor.getPosition() + 0.05);
        } else if (gamepad1.left_bumper) {
            servoMotor.setPosition(servoMotor.getPosition() - 0.05 );
        }

        if(gamepad1.a) {
            bucketMotor.setPower(1.0);
        }

        if(gamepad1.b) {
            bucketMotor.setPower(-1.0);
        }

        //Legacy motors are write-only

        //telemetry.addData("Left Motor", leftMotor.getPower());
        //telemetry.addData("Right Motor", rightMotor.getPower());
        //telemetry.addData("Bucket Motor", bucketMotor.getPower());
        telemetry.addData("Servo", servoMotor.getPosition());
    }
}
