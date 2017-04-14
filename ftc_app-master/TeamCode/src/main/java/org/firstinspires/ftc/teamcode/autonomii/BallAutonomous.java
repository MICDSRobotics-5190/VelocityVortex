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

    @Override
    public void runOpMode() {
        bot = new Robot(hardwareMap);

        MotorPair leftMotors = bot.getTankDrive().getLeftMotors();
        MotorPair rightMotors = bot.getTankDrive().getRightMotors();
        TankDrive drive = bot.getTankDrive();

        // motor config
        bot.getTankDrive().getLeftMotors().setModes(DcMotor.RunMode.RUN_USING_ENCODER);
        bot.getTankDrive().getLeftMotors().setModes(DcMotor.RunMode.RUN_USING_ENCODER);
        bot.getTankDrive().resetEncoders();

        while (opModeIsActive()) {
            bot.getTankDrive().setTargetPosition(FULL_ROTATION);
            bot.getTankDrive().setPower(1);

            // launch ball
            for (int i = 0; i <= 2; i++) {
                bot.getLauncher().fullRotation();
                sleep(750);
            }
        }
    }
}