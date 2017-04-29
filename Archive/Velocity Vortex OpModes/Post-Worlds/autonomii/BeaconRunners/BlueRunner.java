package org.firstinspires.ftc.teamcode.autonomii.BeaconRunners;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.robodata.EncoderValues;

/**
 * Created by amigala on 4/27/2017.
 * blue team goes right. red goes left
 */

@Autonomous(name="BlueRunnerBeacon",group="BeaconRunners")
//@Disabled
public class BlueRunner extends LinearOpMode implements BeaconConfig {
    private Robot bot;
    private ElapsedTime elapsedTime;
    private static int step;
    private static ModernRoboticsI2cGyro gyro;

    @Override
    public void runOpMode() {
        bot = new Robot(hardwareMap);
        step = 0;
        elapsedTime = new ElapsedTime();
        gyro = bot.getGyro();

        bot.getOpticalDistanceSensor().enableLed(true);
        bot.getTankDrive().setModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

            switch (step) {
                case 0 :
                    bot.getTankDrive().setPower(DRIVE_TRAIN_POWER);
                    sleep(1200);
                    bot.stopMoving();
                    step++;
                    break;
                case 1 :
                    if (gyro.getHeading() < 76 || gyro.getHeading() > 340) { // 84 original
                        bot.getTankDrive().getRightMotors().setPower(0.8);
                        bot.getTankDrive().getLeftMotors().setPower(-0.8);
                    }
                    else {
                        step++;
                    }
                    break;
                case 2 :
                    if (bot.getOpticalDistanceSensor().getRawLightDetected() <= LIGHT_DETECTED_THRESH) {
                        bot.getTankDrive().setPower(0.2);
                    }
                    else if (bot.getOpticalDistanceSensor().getRawLightDetected() > LIGHT_DETECTED_THRESH){
                        bot.stopMoving();
                        step++;
                    }
                    break;
                case 3 :
                    // cap the color and then move slider
                    bot.getColorSensor().enableLed(false);
                    bot.colorScan();
                    int[] currentRGB = bot.getRgbValues();

                    if (currentRGB[0] > 2) {
                        telemetry.addData("Color","Blue");
                        bot.getTankDrive().setPower(1);
                        sleep(1000);
                        bot.stopMoving();
                        step++;
                    }
                    else if (currentRGB[2] > 2) {
                        telemetry.addData("Color","Red");
                        bot.getSlider().setPower(-1);
                        sleep(SLIDER_TIME);
                        bot.stopMoving();
                        step++;
                    }
                    else {
                        telemetry.addData("Program", "Not Risking it!");
                        telemetry.addData("Red", currentRGB[0]);
                        telemetry.addData("Green", currentRGB[1]);
                        telemetry.addData("Blue", currentRGB[2]);
                        step++;
                    }
                    break;
                case 4 : // pull forward
                    bot.getTankDrive().setPower(-DRIVE_TRAIN_POWER);
                    sleep(PULL_FORWARD_TIME);
                    bot.stopMoving();
                    step++;
                    break;
                case 5 :
                    bot.getLauncher().getLauncherMotor().setPower(1);
                    sleep(2500);
                    bot.stopMoving();
                    step++;
                    break;
            }
            telemetry.update();
        }
    }
}