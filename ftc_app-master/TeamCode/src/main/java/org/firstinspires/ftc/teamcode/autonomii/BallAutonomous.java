package org.firstinspires.ftc.teamcode.autonomii;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.EncoderValues;
import org.firstinspires.ftc.teamcode.hardware.MotorPair;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.TankDrive;

import com.qualcomm.robotcore.util.Hardware;

/**
 * Created by amigala on 4/13/2017.
 *
 * The Goal of this program is to get
 */

@Autonomous(name="BallAuto", group="Encoder Opmodes")
public class BallAutonomous extends LinearOpMode implements EncoderValues {
    private Robot bot;
    private int step;

    @Override
    public void runOpMode() {
        bot = new Robot(hardwareMap);
        step = 0;

        MotorPair leftMotors = bot.getTankDrive().getLeftMotors();
        MotorPair rightMotors = bot.getTankDrive().getRightMotors();
        TankDrive drive = bot.getTankDrive();

        // motor config
        bot.getTankDrive().getLeftMotors().setModes(DcMotor.RunMode.RUN_USING_ENCODER);
        bot.getTankDrive().getLeftMotors().setModes(DcMotor.RunMode.RUN_USING_ENCODER);
        bot.getTankDrive().resetEncoders();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Step", String.valueOf(step));
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            switch (step) {
                case 0:
                    bot.getTankDrive().setTargetPosition(-LAUNCHER_FULL_ROTATION);
                    bot.getTankDrive().setPower(1);
                    break;
                case 1:
                    sleep(1000);
                    break;
                case 2:
                    // launch ball
                    bot.getLauncher().fullRotation();
                    break;
                case 3:
                    bot.getIntake().takeInBall();
                    bot.getLauncher().fullRotation();
                    sleep(100);
                    break;
                case 4:
                    // intake new ball and shoot it
                    bot.getLauncher().fullRotation();
                    bot.getIntake().stop();
                    break;
            }
            step++;
            telemetry.update();
        }
    }
}