/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Launcher;
import org.firstinspires.ftc.teamcode.robodata.modestates.IntakeModeState;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.TankDrive;
import org.firstinspires.ftc.teamcode.hardware.MotorPair;
import org.firstinspires.ftc.teamcode.robodata.AccessControl;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Driving", group="TeleOp")  // @Autonomous(...) is the other common choice
public class DriverControlled extends OpMode
{
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private Robot bot = null;
    private AccessControl accessControl = new AccessControl();

    private TankDrive tankDrive;
    private MotorPair leftMotors;
    private MotorPair rightMotors;
    private Launcher launcher;

    private IntakeModeState modeState;

    boolean doFullRotation = false;

    private double power;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        bot = new Robot(hardwareMap);
        tankDrive = bot.getTankDrive();

        tankDrive = bot.getTankDrive();
        leftMotors = bot.getTankDrive().getLeftMotors();
        rightMotors = bot.getTankDrive().getRightMotors();
        launcher = bot.getLauncher();

        // modestates
        modeState = new IntakeModeState();

        power = 1;

        telemetry.addData("Status", "Initialized");

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("Status", "Running: " + runtime.toString());

        if (accessControl.isG1Primary()) {
            leftMotors.setPower(gamepad1.left_stick_y * power);
            rightMotors.setPower(gamepad1.right_stick_y * power);
        }
        else if (accessControl.isG2Primary()) {
            leftMotors.setPower(gamepad2.left_stick_y * power);
            rightMotors.setPower(gamepad2.right_stick_y * power);
        }
        else {
            leftMotors.setPower(gamepad1.left_stick_y * power);
            rightMotors.setPower(gamepad1.right_stick_y * power);
        }

        //&& bot.getLauncher().getLauncherMotor().getCurrentPosition() != 0

        telemetry.addData("Launcher", "Position " + launcher.getLauncherMotor().getCurrentPosition());
        telemetry.addData("Left Drivetrain", "Power " + (leftMotors.getBackPower() + leftMotors.getFrontPower()) /2 );
        telemetry.addData("Right Drivetrain", "Power " + (rightMotors.getBackPower() + rightMotors.getFrontPower()) /2 );
        telemetry.addData("Mode State", modeState.getTelemetryStatus());
        telemetry.addData("Access Control", accessControl.getTelemetryState());

        bot.getColorSensor().enableLed(false);
        bot.colorScan();
        int[] currentRGB = bot.getRgbValues();
        telemetry.addData("Red", currentRGB[0]);
        telemetry.addData("Green", currentRGB[1]);
        telemetry.addData("Blue", currentRGB[2]);

        if(gamepad1.a || gamepad2.a){
            doFullRotation = true;
            bot.getLauncher().fullRotation();
        }

        if(doFullRotation) {
            launcher.getLauncherMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            launcher.getLauncherMotor().setPower(1);

            if (launcher.getLauncherMotor().getCurrentPosition() >= launcher.getLauncherMotor().getTargetPosition()){
                launcher.getLauncherMotor().setPower(0);
                doFullRotation = false;
            }

        }

        if (gamepad1.guide || gamepad2.guide) {
            modeState.changeMode();
        }

        if (gamepad1.b || gamepad2.b) {
            bot.stopMoving();
            doFullRotation = false;
        }

        if(gamepad1.x || gamepad2.x){
            power = 1;
        }

        if(gamepad1.y || gamepad2.y){
            power = 0.5;
        }

        if (gamepad1.right_bumper || gamepad2.right_bumper) {
            bot.getIntake().takeInBall();
        } else if (gamepad1.left_bumper || gamepad2.left_bumper) {
            bot.getIntake().purgeBall();
        }
        else if (modeState.isToggle()) {}
        else {
            bot.getIntake().stop();
        }

        if(gamepad1.dpad_left || gamepad2.dpad_left){
            bot.getSlider().setPower(1);
        } else if (gamepad1.dpad_right || gamepad2.dpad_right){
            bot.getSlider().setPower(-1);
        } else {
            bot.getSlider().setPower(0);
        }


        // access control requesting
        if (gamepad1.dpad_up || gamepad2.dpad_up) {
            accessControl.changeAccess();
        }

        /*if (accessControl.isRequesting()) {
            if (gamepad1.dpad_up || gamepad2.dpad_down) {
                accessControl.setRequesting(false);
            }
        }

        if (gamepad1.dpad_down && accessControl.isG1Primary() && accessControl.isRequesting()) {
            accessControl.changeAccess();
            accessControl.setRequesting(false);
        }

        if (gamepad2.dpad_down && accessControl.isG2Primary() && accessControl.isRequesting()) {
            accessControl.changeAccess();
            accessControl.setRequesting(false);
        }*/

        /* Recording code here */
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

        bot.stopMoving();
        bot.getTankDrive().resetEncoders();

    }

}