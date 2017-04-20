package org.firstinspires.ftc.teamcode.autonomii;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.AutonomousState;
import org.firstinspires.ftc.teamcode.hardware.EncoderValues;
import org.firstinspires.ftc.teamcode.hardware.Robot;

/**
 * Created by amigala on 4/20/2017.
 */

/**
 * Goal of this auto is to move forward, shoot a ball, pick up a ball, shoot it
 */
@Autonomous(name = "BallAutoRefactor", group = "Encoder")
public class BallAutoRefactor extends LinearOpMode implements EncoderValues {
    private Robot bot;
    private AutonomousState autonomousState;

    @Override
    public void runOpMode() throws InterruptedException {
        bot = new Robot(hardwareMap);
        autonomousState = new AutonomousState();

        telemetry.addData("Status","Initialized");
        telemetry.update();

        init();

        while(opModeIsActive()) {
            // add telemetry data here
            telemetry.addData("AutoState",autonomousState.getTelemetryState());

            switch (autonomousState.getState()) {
                case 0 :
                    bot.getTankDrive().setTargetPosition(LAUNCHER_FULL_ROTATION);
                    bot.getTankDrive().setPower(1);
                    bot.getTankDrive().resetEncoders();
                    break;
                case 1 :
                    bot.getLauncher().fullRotation();
                    break;
                case 2 :
                    bot.getIntake().takeInBall();
                    sleep(7000);
                    bot.stopMoving();
                    break;
                case 3 :
                    bot.getLauncher().fullRotation();
                    sleep(1000);
                    break;
            }

            autonomousState.incrementState();
            telemetry.update();
        }
    }
}
