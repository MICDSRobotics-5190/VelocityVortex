package org.firstinspires.ftc.teamcode.autonomii.BeaconRunners;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.MotorPair;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.robodata.EncoderValues;

/**
 * Created by amigala on 4/27/2017.
 * blue team goes right. red goes left
 */

@Autonomous(name="RedBeaconEncoder",group="EncoderBeacons")
public class RedBeaconEncoder extends LinearOpMode implements EncoderValues {
    private Robot bot;
    private MotorPair leftMotors;
    private MotorPair rightMotors;

    private ElapsedTime elapsedTime;
    private int step;
    private ModernRoboticsI2cGyro gyro;

    @Override
    public void runOpMode() {
        bot = new Robot(hardwareMap);
        leftMotors = bot.getTankDrive().getLeftMotors();
        rightMotors = bot.getTankDrive().getRightMotors();

        bot.getTankDrive().setModes(DcMotor.RunMode.RUN_USING_ENCODER);

        leftMotors.getMotor1().setMaxSpeed(MAX_SPEED);
        leftMotors.getMotor2().setMaxSpeed(MAX_SPEED);
        rightMotors.getMotor1().setMaxSpeed(MAX_SPEED);
        rightMotors.getMotor2().setMaxSpeed(MAX_SPEED);

        step = 0;
        elapsedTime = new ElapsedTime();
        gyro = bot.getGyro();
        String colorStatus = null;

        bot.getOpticalDistanceSensor().enableLed(true);
        telemetry.addData("Status","Initialized!");
        telemetry.update();

        // start calibrating the gyro.
        telemetry.addData(">", "Gyro Calibrating. Do Not move!");
        telemetry.update();
        gyro.calibrate();

        // make sure the gyro is calibrated.
        while (!isStopRequested() && gyro.isCalibrating())  {
            sleep(50);
            idle();
        }

        telemetry.addData(">", "Gyro Calibrated.  Press Start.");
        telemetry.update();

        waitForStart();

        telemetry.addData("Status","Running");
        telemetry.update();

        while (opModeIsActive()) {
            telemetry.addData("Step",step);
            telemetry.addData("Heading", gyro.getHeading());
            telemetry.addData("Left Positions", bot.getTankDrive().getLeftMotors().getCurrentPositionAverage());

            switch (step) {
                case 0 :
                    bot.getTankDrive().setTargetPosition(3 * FLOOR_TILE);
                    if((Math.abs(leftMotors.getCurrentPositionAverage()) <= leftMotors.getTargetPositionAverage()) ||
                            Math.abs(rightMotors.getCurrentPositionAverage()) <= rightMotors.getTargetPositionAverage()){
                        bot.getTankDrive().setPower(1);
                    } else {
                        bot.stopMoving();
                        sleep(500);
                        step++;
                    }
                    break;
                case 1 :
                    if (gyro.getHeading() > 275 || gyro.getHeading() < 20) {
                        bot.getTankDrive().getRightMotors().setPower(-1);
                        bot.getTankDrive().getLeftMotors().setPower(1);
                    }
                    else {
                        bot.stopMoving();
                        sleep(500);
                        step++;
                    }
                    break;
                case 2 :
                    bot.getTankDrive().setModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    bot.getTankDrive().setModes(DcMotor.RunMode.RUN_USING_ENCODER);

                    bot.getTankDrive().setTargetPosition(2 * FLOOR_TILE);
                    if((Math.abs(leftMotors.getCurrentPositionAverage()) <= leftMotors.getTargetPositionAverage()) ||
                            Math.abs(rightMotors.getCurrentPositionAverage()) <= rightMotors.getTargetPositionAverage()){
                        bot.getTankDrive().setPower(1);
                    } else {
                        bot.stopMoving();
                        sleep(500);
                        step++;
                    }
                    break;
                case 3 :
                    // cap the color and then move slider
                    bot.getColorSensor().enableLed(false);
                    bot.colorScan();
                    int[] currentRGB = bot.getRgbValues();

                    if (currentRGB[2] > 2) {
                        bot.getTankDrive().setPower(0.2);
                        sleep(1000);
                        bot.stopMoving();
                        colorStatus = "red";
                    }
                    else if (currentRGB[0] > 2) {
                        bot.getSlider().setPower(-0.2);
                        sleep(2000); //slider time
                        bot.stopMoving();
                        colorStatus = "blue";
                    }
                    else {
                        telemetry.addData("Program", "Not Risking it!");
                        telemetry.addData("Red", currentRGB[0]);
                        telemetry.addData("Green", currentRGB[1]);
                        telemetry.addData("Blue", currentRGB[2]);
                        colorStatus = "ah shoot dang it";
                    }
                    bot.stopMoving();
                    sleep(500);
                    step++;
                    break;
                default:
                    telemetry.addData("Step", "Done!");
                    telemetry.addData("Color Detected", colorStatus);
            }
            telemetry.update();
        }
    }
}