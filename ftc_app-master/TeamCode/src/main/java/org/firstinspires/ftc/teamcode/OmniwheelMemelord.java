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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * Now abandoned, this is where the code to run the omniwheel drive is.
 * It worked fairly decently (with some issues on the diagonals and with
 * the joysticks), but in the end it was scrapped because the omniwheels
 * in particular had tons of trouble going up the ramp, no matter
 * how they were aligned.
 * Maybe someday we'll have the time, the reason, and the parts to experiment
 * with this again later, or we could for fun :)
 */


@TeleOp(name="Omniwheel Driver", group="Driver-Controlled OpModes")  // @Autonomous(...) is the other common choice
@Disabled
public class OmniwheelMemelord extends OpMode
{
    // Declare Objects (Hardware & Other)
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftFrontMotor = null;
    private DcMotor leftBackMotor = null;
    private DcMotor rightFrontMotor = null;
    private DcMotor rightBackMotor = null;

    private String approximateDirection = "";


    //Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
         /* Initialize the hardware variables. The strings must
        correspond to the names in the configuration file. */
        leftFrontMotor  = hardwareMap.dcMotor.get("left front motor");
        leftBackMotor = hardwareMap.dcMotor.get("left back motor");
        rightFrontMotor = hardwareMap.dcMotor.get("right front motor");
        rightBackMotor = hardwareMap.dcMotor.get("right back motor");

        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightBackMotor.setPower(0);

        // eg: Set the drive motor directions:
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        telemetry.addData("Status", "Initialized");
    }


    //Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
    @Override
    public void init_loop() {

    }


    //Code to run ONCE when the driver hits PLAY
    @Override
    public void start() {
        runtime.reset();
    }


    //Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {
        telemetry.addData("Status", "Running: " + runtime.toString());
        telemetry.addData("X Direction: ", + (-gamepad1.left_stick_x));
        telemetry.addData("Y Direction: ", + (-gamepad1.left_stick_y));

        //Jack's amazing swag with joysticks
        double xg = gamepad1.right_stick_x, yg = gamepad1.right_stick_y; //displacement of the joystick
        double power_left_up, power_right_up;
        if (xg > 0) {
            power_left_up = Math.sqrt(2) * ((xg + yg) / 2);
        } else {
            power_left_up = - Math.sqrt(2) * ((xg + yg) / 2);
        }
        if (yg > 0) {
            power_right_up = Math.sqrt(2) * ((yg - xg) / 2);
        } else {
            power_right_up = - Math.sqrt(2) * ((yg - xg) / 2);
        }
        double power_left_down = power_right_up;
        double power_right_down = power_left_up;
        leftFrontMotor.setPower(power_left_up);
        leftBackMotor.setPower(power_left_down);
        rightFrontMotor.setPower(power_right_up);
        rightBackMotor.setPower(power_right_down);

        //BLAKE'S okish TRASH
        if (gamepad1.dpad_up || gamepad1.dpad_right || gamepad1.dpad_down || gamepad1.dpad_left) {

            if (gamepad1.dpad_right) {
                if (gamepad1.dpad_down) {
                    leftFrontMotor.setPower(0);
                    leftBackMotor.setPower(1);
                    rightFrontMotor.setPower(1);
                    rightBackMotor.setPower(0);
                } else if (gamepad1.dpad_up) {
                    leftFrontMotor.setPower(1);
                    leftBackMotor.setPower(0);
                    rightFrontMotor.setPower(0);
                    rightBackMotor.setPower(1);
                } else {
                    leftFrontMotor.setPower(1);
                    leftBackMotor.setPower(1);
                    rightFrontMotor.setPower(1);
                    rightBackMotor.setPower(1);
                }
            } else if (gamepad1.dpad_up) {
                leftFrontMotor.setPower(1);
                leftBackMotor.setPower(-1);
                rightFrontMotor.setPower(-1);
                rightBackMotor.setPower(1);
            } else if (gamepad1.dpad_down) {
                leftFrontMotor.setPower(-1);
                leftBackMotor.setPower(1);
                rightFrontMotor.setPower(1);
                rightBackMotor.setPower(-1);
            }

            if (gamepad1.dpad_left) {
                if (gamepad1.dpad_down) {
                    leftFrontMotor.setPower(0);
                    leftBackMotor.setPower(-1);
                    rightFrontMotor.setPower(-1);
                    rightBackMotor.setPower(0);
                } else if (gamepad1.dpad_up) {
                    leftFrontMotor.setPower(-1);
                    leftBackMotor.setPower(0);
                    rightFrontMotor.setPower(0);
                    rightBackMotor.setPower(-1);
                } else {
                    leftFrontMotor.setPower(-1);
                    leftBackMotor.setPower(-1);
                    rightFrontMotor.setPower(-1);
                    rightBackMotor.setPower(-1);
                }
            }  else if (gamepad1.dpad_up) {
                leftFrontMotor.setPower(1);
                leftBackMotor.setPower(-1);
                rightFrontMotor.setPower(-1);
                rightBackMotor.setPower(1);
            } else if (gamepad1.dpad_down) {
                leftFrontMotor.setPower(-1);
                leftBackMotor.setPower(1);
                rightFrontMotor.setPower(1);
                rightBackMotor.setPower(-1);
            }

        } else {
            leftFrontMotor.setPower(0);
            leftBackMotor.setPower(0);
            rightFrontMotor.setPower(0);
            rightBackMotor.setPower(0);
        }

    }

    /* Code to run ONCE after the driver hits STOP */
    @Override
    public void stop() {

        //Make sure the motors are stopped, yep
        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightBackMotor.setPower(0);

    }

}
