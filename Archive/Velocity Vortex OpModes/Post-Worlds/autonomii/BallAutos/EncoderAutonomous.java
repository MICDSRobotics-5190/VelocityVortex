package org.firstinspires.ftc.teamcode.autonomii.BallAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.MotorPair;
import org.firstinspires.ftc.teamcode.robodata.AutonomousState;
import org.firstinspires.ftc.teamcode.robodata.EncoderValues;
import org.firstinspires.ftc.teamcode.hardware.Robot;

/**
 * Created by BAbel on 4/25/2017.
 */

//@Autonomous(name = "EncoderAuto", group = "Encoder")
public class EncoderAutonomous extends LinearOpMode implements EncoderValues {

    private Robot bot;
    private MotorPair leftMotors;
    private MotorPair rightMotors;

    private ElapsedTime runtime = new ElapsedTime();

    private int step;

    @Override
    public void runOpMode(){
        bot = new Robot(hardwareMap);
        leftMotors = bot.getTankDrive().getLeftMotors();
        rightMotors = bot.getTankDrive().getRightMotors();

        bot.getTankDrive().setModes(DcMotor.RunMode.RUN_USING_ENCODER);

        leftMotors.getMotor1().setMaxSpeed(MAX_SPEED);
        leftMotors.getMotor2().setMaxSpeed(MAX_SPEED);
        rightMotors.getMotor1().setMaxSpeed(MAX_SPEED);
        rightMotors.getMotor2().setMaxSpeed(MAX_SPEED);

        step = 0;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while(opModeIsActive()){

            switch(step){
                case 0:
                    bot.getTankDrive().setTargetPosition(FLOOR_TILE);
                    if((Math.abs(leftMotors.getCurrentPositionAverage()) <= leftMotors.getTargetPositionAverage()) ||
                            Math.abs(rightMotors.getCurrentPositionAverage()) <= rightMotors.getTargetPositionAverage()){
                        bot.getTankDrive().setPower(-1);
                    } else {
                        bot.stopMoving();
                        sleep(500);
                        step = 1;
                    }
                    break;
                case 1:
                    bot.getLauncher().getLauncherMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    bot.getLauncher().fullRotation();
                    bot.getLauncher().getLauncherMotor().setPower(1);
                    sleep(1000);
                    bot.stopMoving();
                    step = 2;
                    break;
                case 2:
                    bot.getIntake().takeInBall();
                    sleep(2000);
                    bot.stopMoving();
                    step = 3;
                    break;
                case 3:
                    bot.getLauncher().getLauncherMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    bot.getLauncher().fullRotation();
                    bot.getLauncher().getLauncherMotor().setPower(1);
                    sleep(1000);
                    bot.stopMoving();
                    step = 4;
                    break;
                case 4:
                    bot.getTankDrive().setTargetPosition(4 * FLOOR_TILE);
                    if((Math.abs(leftMotors.getCurrentPositionAverage()) <= leftMotors.getTargetPositionAverage()) ||
                            Math.abs(rightMotors.getCurrentPositionAverage()) <= rightMotors.getTargetPositionAverage()){
                        bot.getTankDrive().setPower(-1);
                    } else {
                        bot.stopMoving();
                        sleep(500);
                        step = 5;
                    }
                    break;
                case 5:
                    telemetry.addData("STATUS", "Completed");
                    break;
                default:
                    break;
            }

            telemetry.addData("Runtime", runtime.seconds() + "s");
            telemetry.addData("Step", step);
            telemetry.addData("Left Drivetrain", "Power " + (leftMotors.getBackPower() + leftMotors.getFrontPower()) /2 );
            telemetry.addData("Right Drivetrain", "Power " + (rightMotors.getBackPower() + rightMotors.getFrontPower()) /2 );

            telemetry.addData("Encoders (Left)", leftMotors.getCurrentPositionAverage());
            telemetry.addData("Encoders (Right)", rightMotors.getCurrentPositionAverage());
            telemetry.update();

        }


    }


}
