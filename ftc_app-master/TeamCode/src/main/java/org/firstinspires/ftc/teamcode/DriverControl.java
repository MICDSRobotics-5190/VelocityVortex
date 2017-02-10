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
import com.qualcomm.robotcore.util.Hardware;


/*
 * This file is the iterative (non-linear) Op-Mode made for the driver controlled
 * TeleOp period of an FTC match for Technoramic, Team 5190, in 2016-2017.
 *
 * The bot this was made for had an awesome west coast drivetrain, used motor encoders for once,
 * <insert cool stuff about finished bot>, yep
 */

@TeleOp(name="Drive", group="Driver-Controlled OpModes")  // @Autonomous(...) is the other common choice
//@Disabled
public class DriverControl extends OpMode
{
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    private Robot dan = new Robot();

    private boolean driverOneSharing;
    private boolean driverTwoSharing;



    /* Code to run ONCE when the driver hits INIT */
    @Override
    public void init() {

        /* Initialize the hardware variables. The strings must
        correspond to the names in the configuration file. */

        dan.setupHardware(hardwareMap);

        driverOneSharing = false;
        driverTwoSharing = false;

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
        telemetry.addData("Right Motor", dan.rightMotor.getPower());
        telemetry.addData("Left Motor", dan.leftMotor.getPower());
        telemetry.addData("Spinner", dan.spinner.getPower());
        telemetry.addData("Flywheel", dan.flywheel.getPower());
        telemetry.addData("Beacon Hitter", dan.beaconSlider.getPower());

        if(gamepad1.x){
            driverOneSharing = false;
        }
        if(gamepad1.y){
            driverOneSharing = true;
        }

        if(gamepad2.x){
            driverTwoSharing = false;
        }
        if(gamepad2.y){
            driverTwoSharing = true;
        }

        if(driverOneSharing && driverTwoSharing){
            if(gamepad1.dpad_up || gamepad2.dpad_up){
                dan.leftMotor.setDirection(DcMotor.Direction.REVERSE);
                dan.rightMotor.setDirection(DcMotor.Direction.FORWARD);
            }
            if(gamepad1.dpad_down || gamepad2.dpad_down){
                dan.leftMotor.setDirection(DcMotor.Direction.FORWARD);
                dan.rightMotor.setDirection(DcMotor.Direction.REVERSE);
            }

            dan.leftMotor.setPower(-(gamepad1.left_stick_y + gamepad2.left_stick_y) / 2);
            dan.rightMotor.setPower(-(gamepad1.right_stick_y + gamepad2.right_stick_y) / 2);
        } else if(driverOneSharing && !driverTwoSharing){
            if(gamepad2.dpad_up){
                dan.leftMotor.setDirection(DcMotor.Direction.REVERSE);
                dan.rightMotor.setDirection(DcMotor.Direction.FORWARD);
            }
            if(gamepad2.dpad_down){
                dan.leftMotor.setDirection(DcMotor.Direction.FORWARD);
                dan.rightMotor.setDirection(DcMotor.Direction.REVERSE);
            }

            dan.leftMotor.setPower(-gamepad2.left_stick_y);
            dan.rightMotor.setPower(-gamepad2.right_stick_y);
        } else if(!driverOneSharing && driverTwoSharing){
            if(gamepad1.dpad_up){
                dan.leftMotor.setDirection(DcMotor.Direction.REVERSE);
                dan.rightMotor.setDirection(DcMotor.Direction.FORWARD);
            }
            if(gamepad1.dpad_down){
                dan.leftMotor.setDirection(DcMotor.Direction.FORWARD);
                dan.rightMotor.setDirection(DcMotor.Direction.REVERSE);
            }

            dan.leftMotor.setPower(-gamepad1.left_stick_y);
            dan.rightMotor.setPower(-gamepad1.right_stick_y);
        } else {
            if(gamepad1.dpad_up){
                dan.leftMotor.setDirection(DcMotor.Direction.REVERSE);
                dan.rightMotor.setDirection(DcMotor.Direction.FORWARD);
            }
            if(gamepad1.dpad_down){
                dan.leftMotor.setDirection(DcMotor.Direction.FORWARD);
                dan.rightMotor.setDirection(DcMotor.Direction.REVERSE);
            }

            dan.leftMotor.setPower(-gamepad1.left_stick_y);
            dan.rightMotor.setPower(-gamepad1.right_stick_y);
        }


        if(gamepad1.left_bumper){
            dan.spinner.setPower(1);
        } else if (gamepad1.right_bumper){
            dan.spinner.setPower(-1);
        } else {
            dan.spinner.setPower(0);
        }

        if(gamepad1.a){
            dan.flywheel.setPower(1);
        }

        if(gamepad1.b) {
            dan.flywheel.setPower(0);
        }

        if(gamepad2.dpad_left){
            dan.beaconSlider.setPower(1);
        } else if (gamepad2.dpad_right){
            dan.beaconSlider.setPower(-1);
        } else {
            dan.beaconSlider.setPower(0);
        }

        if(gamepad1.start || gamepad2.start){
            dan.stopMoving();
        }

    }


    /* Code to run ONCE after the driver hits STOP */
    @Override
    public void stop() {
        dan.stopMoving();
    }

}
