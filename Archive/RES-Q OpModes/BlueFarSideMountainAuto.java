package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;

public class BlueFarSideMountainAuto extends LinearOpMode {

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

    //Write this when making an autonomous program
    @Override
    public void runOpMode() throws InterruptedException {

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

        //Take a wild guess bud
        waitForStart();

        //Time it takes to move 1 floor block (or 60 cm) below, calculations based off of that.
        //Reminder that these are in ms FIX BLAKE BLAEK BLACK PLS
        long blockTime = 2280;
        long farBlockTime = 9673;

        //Time it takes to turn a quarter-turn (in-place) below, calculations based off of that.
        //Reminder that these are in ms FIX BLAKE BLAEK BLACK
        long turnTime = 4750;
        //half of this is...
        long halfTurnTime = 2375;

        //boolean gotColor = false;
        //int i = 0;

        telemetry.addData("Status", "Running!");

        /*
        telemetry.addData("Light Sensed", sensorLight.getLightDetected());
        //Change to whatever value corresponds to blue
        if(sensorLight.getLightDetected() < 0.35){
            telemetry.addData("Color", "Red");
        }
        //Change to whatever value corresponds to red
        if(sensorLight.getLightDetected() > 0.28){
            telemetry.addData("Color", "Blue");
        } */
        
        //blue
        sleep(10000);
        forward(blockTime);
        turnLeft(halfTurnTime);
        forward(farBlockTime);
        //turnLeft(halfTurnTime);
        forward(blockTime*2);

        //Stop moving after
        leftMotor.setPower(0);
        rightMotor.setPower(0);

        telemetry.addData("Status", "Completed!");

    }

    public void forward(long blockTime) throws InterruptedException{
        leftMotor.setPower(1.0);
        rightMotor.setPower(1.0);
        sleep(blockTime);
    }

    public void turnRight(long turnTime) throws InterruptedException{
        leftMotor.setPower(1.0);
        rightMotor.setPower(-1.0);
        sleep(turnTime);
    }

    public void turnLeft(long turnTime) throws InterruptedException{
        leftMotor.setPower(-1.0);
        rightMotor.setPower(1.0);
        sleep(turnTime);
    }

}
