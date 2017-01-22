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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/*
 * This file is the iterative (non-linear) Op-Mode made for the driver controlled
 * TeleOp period of an FTC match for Technoramic, Team 5190, in 2016-2017.
 *
 * The bot this was made for had an awesome west coast drivetrain, used motor encoders for once,
 * <insert cool stuff about finished bot>, yep
 */

@TeleOp(name="Drive", group="Driver-Controlled OpModes")  // @Autonomous(...) is the other common choice
//@Disabled
public class DadDriver extends OpMode
{
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftMotor = null;
    private DcMotor rightMotor = null;

    private DcMotor spinner = null;
    private DcMotor shooter = null;

    // uninplemented claw objects
    private CRServo leftClaw = null;
    private CRServo rightClaw = null;

    // shooter variables
    private double endTime = 0;
    private final double revolutionTime = 0.349;



    /* Code to run ONCE when the driver hits INIT */
    @Override
    public void init() {

        /* Initialize the hardware variables. The strings must
        correspond to the names in the configuration file. */
        leftMotor  = hardwareMap.dcMotor.get("left motor");
        rightMotor = hardwareMap.dcMotor.get("right motor");

        shooter = hardwareMap.dcMotor.get("shooter");
        spinner = hardwareMap.dcMotor.get("spinner");

        //leftClaw = hardwareMap.crservo.get("left claw");
        //rightClaw = hardwareMap.crservo.get("right claw");

        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        spinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // Reverse the motor that runs backwards when connected directly to the battery
        leftMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors

        spinner.setDirection(DcMotor.Direction.FORWARD);
        shooter.setDirection(DcMotor.Direction.FORWARD);

        //leftClaw.setDirection(CRServo.Direction.FORWARD);
        //rightClaw.setDirection(CRServo.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
    }


    /* Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY */
    @Override
    public void init_loop() {
    }


    /* Code to run ONCE when the driver hits PLAY */
    @Override
    public void start() {
        runtime.reset();
    }


    /* Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP */
    @Override
    public void loop() {
        telemetry.addData("Status", "Running: " + runtime.toString());
        telemetry.addData("Right Motor", rightMotor.getPower());
        telemetry.addData("Left Motor", leftMotor.getPower());
        telemetry.addData("Spinner", spinner.getPower());
        //telemetry.addData("Left Claw", leftClaw.getPower());
        //telemetry.addData("Right Claw", rightClaw.getPower());


        // Drivetrain code (note: The joystick goes negative when pushed forwards)
        leftMotor.setPower(-gamepad1.left_stick_y);
        rightMotor.setPower(-gamepad1.right_stick_y);

        if(gamepad1.left_bumper){
            spinner.setPower(1);
        } else if (gamepad1.right_bumper){
            spinner.setPower(-1);
        } else {
            spinner.setPower(0);
        }

        // shooter code
        if(gamepad1.a) {
            endTime = getRuntime() + revolutionTime;
        }

        if(getRuntime() < endTime){
            shooter.setPower(1);
        } else {
            shooter.setPower(0);
        }

        /* unimplemented claw objects
        if (gamepad2.a){
            leftClaw.setPower(0.5);
            rightClaw.setPower(0.5);
        }

        if (gamepad2.b) {
            clawMotorsLow();
        }

        if (gamepad2.x){
            leftClaw.setPower(-0.5);
            rightClaw.setPower(-0.5);
        }

        if(gamepad2.right_trigger >= 0.1) {
            leftClaw.setPower(gamepad2.right_trigger);
            rightClaw.setPower(gamepad2.right_trigger);
        }

        if(gamepad2.left_trigger >= 0.1) {
            leftClaw.setPower(-gamepad2.left_trigger);
            rightClaw.setPower(-gamepad2.left_trigger);
        }
        */
        /* claw controls
        if (gamepad2.dpad_up){
            leftClaw.setPower(1);
            rightClaw.setPower(1);
        }
        if (gamepad2.dpad_down) {
            leftClaw.setPower(1);
            rightClaw.setPower(1);
        }
        if (gamepad2.x) {
            leftClaw.setPower(0);
            rightClaw.setPower(0);
        }
        */
    }


    /* Code to run ONCE after the driver hits STOP */
    @Override
    public void stop() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        spinner.setPower(0);
        shooter.setPower(0);
    }

    private void clawMotorsLow() {
        leftClaw.setPower(0);
        rightClaw.setPower(0);
    }

}
