package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

public class LinearPopsicleDrive extends LinearOpMode {

    DcMotor leftMotor;
    DcMotor rightMotor;


    //Write this when making an autonomous program
    @Override
    public void runOpMode() throws InterruptedException {


        //Setting up the motors
        leftMotor = hardwareMap.dcMotor.get("left_drive");
        rightMotor = hardwareMap.dcMotor.get("right_drive");
        rightMotor.setDirection(DcMotor.Direction.REVERSE);


        //Take a wild guess bud
        waitForStart();

        //Setting the motors to go at different speeds. Sleep is to wait for x milliseconds.
        leftMotor.setPower(1.0);
        rightMotor.setPower(1.0);
        sleep(1000);
        leftMotor.setPower(0.5);
        rightMotor.setPower(0.5);
        sleep(500);
        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);
        sleep(250);


        }

    }
