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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

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

@Autonomous(name="Blue - Beacon", group="ABlue OpMode")  // @Autonomous(...) is the other common choice
//@Disabled
public class BlueTeamUnbeacon extends LinearVisionOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftMotor = null;
    private DcMotor rightMotor = null;
    private DcMotor spinner = null;

    /*Declaring constant values */
    final int MOTOR_PULSE_PER_REVOLUTION = 7;
    final int MOTOR_GEAR_RATIO = 80;
    final int FULL_REVOLUTION = 1200;
    final int FLOOR_BLOCK = 2292;
    //final int FLOOR_TILE = x;
    //final int QUARTER_TURN = x;

    //Parts of the autonomous program
    int step = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        /* Initialize the hardware variables. The strings must
        correspond to the names in the configuration file. */
        leftMotor  = hardwareMap.dcMotor.get("left motor");
        rightMotor = hardwareMap.dcMotor.get("right motor");
        spinner = hardwareMap.dcMotor.get("spinner");

        // Set the drive motor directions
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);
        spinner.setDirection(DcMotor.Direction.FORWARD);

        //Prepare the encoders to be used
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        boolean blueLeft = false;
        boolean redLeft = false;
        boolean blueRight = false;
        boolean redRight = false;

        telemetry.addData("Status", "Initialized!");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //leftMotor.setMaxSpeed(MOTOR_PULSE_PER_REVOLUTION * MOTOR_GEAR_RATIO);
            //rightMotor.setMaxSpeed(MOTOR_PULSE_PER_REVOLUTION * MOTOR_GEAR_RATIO);

            //Setting up some output for the user to see. (Usually for troubleshooting)
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Step", step);
            telemetry.addData("Right Motor Posn", leftMotor.getCurrentPosition());
            telemetry.addData("Left Motor Posn", rightMotor.getCurrentPosition());

            if(step == 1) {

                leftMotor.setPower(1);
                rightMotor.setPower(1);

                leftMotor.setTargetPosition(FLOOR_BLOCK);
                rightMotor.setTargetPosition(FLOOR_BLOCK);

                if (leftMotor.getCurrentPosition() >= leftMotor.getTargetPosition() - 20 && leftMotor.getCurrentPosition() <= leftMotor.getTargetPosition() + 20) {
                    leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    step = 2;
                }

            } else if (step == 2) {

                leftMotor.setPower(-1);
                rightMotor.setPower(-1);

                leftMotor.setTargetPosition(FULL_REVOLUTION);
                rightMotor.setTargetPosition(FULL_REVOLUTION);

                if(leftMotor.getCurrentPosition() >= leftMotor.getTargetPosition() - 20 && leftMotor.getCurrentPosition() <= leftMotor.getTargetPosition() + 20){
                    leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    step = 3;
                }

            } else if (step == 3){

                leftMotor.setPower(0);
                rightMotor.setPower(0);

            } else if (step == 4){

            }

            telemetry.update();

            //idle(); // OpenCV broke idle, we could troubleshoot later. Basically check LinearOpMode and LinearVisionOpmode.
        }
    }

    //Necessary for using Vuforia and outputting location matrixes.
    String format(OpenGLMatrix transformationMatrix) {
        return transformationMatrix.formatAsTransform();
    }

}
