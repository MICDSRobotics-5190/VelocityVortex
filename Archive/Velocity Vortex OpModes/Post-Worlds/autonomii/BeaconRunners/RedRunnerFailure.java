package org.firstinspires.ftc.teamcode.autonomii.BeaconRunners;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Robot;

/**
 * Created by amigala on 4/27/2017.
 */

//@Autonomous(name="RedRunnerBeacon", group="Beacons")
public abstract class RedRunnerFailure extends LinearOpMode implements BeaconConfig {
    private Robot bot;
    private ElapsedTime elapsedTime;
    private int step;

    /*
    @Override
    public void runOpMode() {
        elapsedTime = new ElapsedTime();
        elapsedTime.startTime();

        bot = new Robot(hardwareMap);
        step = 0;

        bot.getTankDrive().setModes(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        elapsedTime.reset();


        waitForStart();
        telemetry.addData("Time (s)", elapsedTime.seconds());

        while (opModeIsActive()) {
            // telemetry
            telemetry.addData("Step",step);

            // robot start
            switch (step) {
                case 0 : // move until we see the light
                while (bot.getOpticalDistanceSensor().getLightDetected() != LIGHT_DETECTED_THRESH) {
                    bot.getTankDrive().setPower(0.2);
                }
                bot.stopMoving();
                break;

                case 1 :
                    // we are now at the beacon, what happens next?
                    // rotate left side to position
                    bot.getTankDrive().getRightMotors().setPower(0.2);
                    sleep(TURN_LEFT_TIME);
                    bot.stopMoving();
                    break;

                case 2 : // get the colors; this is read team
                    while (true) {
                        bot.getColorSensor().enableLed(false);
                        bot.colorScan();
                        int[] currentRGB = bot.getRgbValues();
                        telemetry.addData("Red", currentRGB[0]);
                        telemetry.addData("Green", currentRGB[1]);
                        telemetry.addData("Blue", currentRGB[2]);

                        if (currentRGB[0] < RGB_VALUE_THRESH || currentRGB[1] < RGB_VALUE_THRESH || currentRGB[2] < RGB_VALUE_THRESH) {
                            bot.getTankDrive().setPower(0.2);
                        }
                        else {
                            bot.stopMoving();
                            telemetry.clearAll();
                            telemetry.addData("Step",step);
                            break;
                        }
                    }
                    break;

                case 3 : // move beacon slider
                    bot.getColorSensor().enableLed(false);
                    bot.colorScan();
                    int[] currentRGB = bot.getRgbValues();

                    if (currentRGB[0] > 2) {
                        bot.getTankDrive().setPower(0.2);
                        sleep(1);
                        bot.stopMoving();
                        idle();
                    }
                    if (currentRGB[2] > 2) {
                        bot.getSlider().setPower(0.2);
                        sleep(SLIDER_TIME);
                        bot.stopMoving();
                        idle();
                    }
                    else {
                        telemetry.addData("Program", "Not Risking it!");
                        telemetry.addData("Red", currentRGB[0]);
                        telemetry.addData("Green", currentRGB[1]);
                        telemetry.addData("Blue", currentRGB[2]);
                        idle();
                    }
            }

            step++;
            telemetry.update();
        }
    }*/
}