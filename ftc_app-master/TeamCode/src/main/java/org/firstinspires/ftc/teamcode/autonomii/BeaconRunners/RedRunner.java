package org.firstinspires.ftc.teamcode.autonomii.BeaconRunners;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.robodata.EncoderValues;

/**
 * Created by amigala on 4/27/2017.
 */

@Autonomous(name="RedRunnerBeacon",group="BeaconRunners")
public class RedRunner extends LinearOpMode implements BeaconConfig {
    private Robot bot;
    private ElapsedTime elapsedTime;
    private int step;

    @Override
    public void runOpMode() {
        bot = new Robot(hardwareMap);
        step = 0;
        elapsedTime = new ElapsedTime();

        bot.getOpticalDistanceSensor().enableLed(true);
        bot.getTankDrive().setModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("Status","Initialized!");
        telemetry.update();

        waitForStart();

        telemetry.addData("Status","Running");
        telemetry.update();

        while (opModeIsActive()) {
            telemetry.addData("Step",step);

            switch (step) {
                case 0 :
                    bot.getTankDrive().setPower(DRIVE_TRAIN_POWER);
                    sleep(1010);
                    bot.stopMoving();
                    step++;
                    break;
                case 1 :
                    bot.getTankDrive().getRightMotors().setPower(DRIVE_TRAIN_POWER);
                    bot.getTankDrive().getLeftMotors().setPower(-DRIVE_TRAIN_POWER);
                    sleep(ROTATION_TIME);
                    bot.stopMoving();
                    step++;
                    break;
                case 2 :
                    if (bot.getOpticalDistanceSensor().getRawLightDetected() != LIGHT_DETECTED_THRESH) {
                        bot.getTankDrive().setPower(0.2);
                    }
                    else {
                        bot.stopMoving();
                        step++;
                        break;
                    }
                case 3 :
                    // cap the color and then move slider
                    bot.getColorSensor().enableLed(false);
                    bot.colorScan();
                    int[] currentRGB = bot.getRgbValues();

                    if (currentRGB[0] > 2) {
                        bot.getTankDrive().setPower(0.2);
                        sleep(1000);
                        bot.stopMoving();
                        step++;
                        break;
                    }
                    else if (currentRGB[2] > 2) {
                        bot.getSlider().setPower(0.2);
                        sleep(SLIDER_TIME);
                        bot.stopMoving();
                        step++;
                        break;
                    }
                    else {
                        telemetry.addData("Program", "Not Risking it!");
                        telemetry.addData("Red", currentRGB[0]);
                        telemetry.addData("Green", currentRGB[1]);
                        telemetry.addData("Blue", currentRGB[2]);
                        step++;
                        break;
                    }
                case 4 : // pull forward
                    bot.getTankDrive().setPower(-1);
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