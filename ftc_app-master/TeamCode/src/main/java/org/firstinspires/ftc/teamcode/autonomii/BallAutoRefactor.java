package org.firstinspires.ftc.teamcode.autonomii;

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
    private static AutonomousState autonomousState;

    @Override
    public void runOpMode() throws InterruptedException {
        bot = new Robot(hardwareMap);
        autonomousState = new AutonomousState();

        // bot init
        bot.getTankDrive().setModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bot.getLauncher().getLauncherMotor().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bot.getIntake().getIntakeMotor().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status","Initialized");
        telemetry.update();

        //init();

        waitForStart();

        while(opModeIsActive()) {
            // add telemetry data here
            telemetry.addData("AutoState",autonomousState.getTelemetryState());

            autonomousState.setState(-1);
            autonomousState.incrementState();


            switch (autonomousState.getState()) {
                case 0 :
                    bot.getTankDrive().setTargetPosition(FULL_ROTATION);
                    bot.getTankDrive().setPower(-1);
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
                case 4 :
                    autonomousState.setState(0);
                    throw new InterruptedException();
            }

            telemetry.update();
        }
    }
}
