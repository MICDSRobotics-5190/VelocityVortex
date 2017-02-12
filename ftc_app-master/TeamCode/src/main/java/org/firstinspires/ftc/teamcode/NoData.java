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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;

/*
 * This file is the linear Op-Mode made for the non-driver controlled
 * Autonomous period of an FTC match for Technoramic, Team 5190, in 2016-2017.
 *
 * The bot this was made for had an awesome west coast drivetrain, used motor encoders for once,
 * could use the camera to easily find the beacons and locate itself on the field, and
 * hit the button to score tons of points and release more balls into the field.
 * The motor encoders helped a lot in precise movement and getting on the ramp.
 */

@Autonomous(name="No Beacon", group="ANo Beacon OpMode")  // @Autonomous(...) is the other common choice
//@Disabled
public class NoData extends LinearOpMode {

    /* Declare OpMode hardware. */
    private ElapsedTime runtime = new ElapsedTime();

    private Robot dan = new Robot();

    /*Declaring constant values */
    final int MOTOR_PULSE_PER_REVOLUTION = 7;
    final int MOTOR_GEAR_RATIO = 80;
    final int FULL_REVOLUTION = 1200;
    final int FLOOR_BLOCK = 2300;
    //final int QUARTER_TURN = x;

    @Override
    public void runOpMode() throws InterruptedException {

        /* Initialize the hardware variables. The strings must
        correspond to the names in the configuration file. */

        dan.setupHardware(hardwareMap);

        /* Prepare the encoders to be used */
        dan.resetEncoders();

        telemetry.addData("Status", "Initialized!");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        runtime.reset();

        telemetry.addData("Status", "Run Time: " + runtime.toString());

        //Set the target distance to travel (to the ball)
        dan.leftMotor.setTargetPosition((int)(2.5 * FLOOR_BLOCK));
        dan.rightMotor.setTargetPosition((int)(2.5 * FLOOR_BLOCK));

        telemetry.addData("Encoders", dan.leftMotor.getTargetPosition());
        telemetry.update();
        sleep(20000);

        //Run to the ball until it gets to the target distance.
        while (!(dan.leftMotor.getCurrentPosition() >= dan.leftMotor.getTargetPosition() - 10 && dan.leftMotor.getCurrentPosition() <= dan.leftMotor.getTargetPosition() + 10)) {
            dan.drivetrainPower(1);

            telemetry.addData("Right Motor Posn", dan.leftMotor.getCurrentPosition());
            telemetry.addData("Left Motor Posn", dan.rightMotor.getCurrentPosition());
            telemetry.update();
        }

        //Stop
        dan.stopMoving();

        telemetry.update();
    }

}
