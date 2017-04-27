package org.firstinspires.ftc.teamcode.autonomii.BallAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robodata.AutonomousState;
import org.firstinspires.ftc.teamcode.robodata.EncoderValues;
import org.firstinspires.ftc.teamcode.hardware.Robot;

/**
 * Created by amigala on 4/20/2017.
 */

/**
 * Goal of this auto is to move forward, shoot a ball, pick up a ball, shoot it
 */
@Autonomous(name = "BallAutoRefactor", group = "Encoder")
public class BallAutoRefactor extends LinearOpMode implements EncoderValues {
    private static Robot bot;
    private static int step;

    @Override
    public void runOpMode() throws InterruptedException {
        bot = new Robot(hardwareMap);
        bot.getTankDrive().setModes(DcMotor.RunMode.RUN_USING_ENCODER);
        step = 0;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // move into position
        //bot.getTankDrive().setTargetPosition(FULL_ROTATION);
        bot.getTankDrive().setPower(-1);
        sleep(400);
        bot.stopMoving();


        sleep(1000);
        bot.stopMoving();

        // launch ball
        telemetry.addData("Status", "Launch");
        sleep(1000);
        telemetry.update();

        bot.getLauncher().getLauncherMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bot.getLauncher().fullRotation();
        bot.getLauncher().getLauncherMotor().setPower(1);
        sleep(3000);
        bot.stopMoving();

        // take in a new ball
        telemetry.addData("Status","Taking in ball...");
        telemetry.update();
        // take in a new ball
        bot.getIntake().takeInBall();
        sleep(7000);
        bot.stopMoving();

        // launch a new ball
        telemetry.addData("Status", "Launch");
        telemetry.update();
        sleep(1000);

        bot.getLauncher().getLauncherMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bot.getLauncher().fullRotation();
        bot.getLauncher().getLauncherMotor().setPower(1);
        sleep(3000);
        bot.stopMoving();

        // ram the big ball
        bot.getTankDrive().setPower(-1);
        sleep(800);
        bot.stopMoving();

        idle();
    }
}
