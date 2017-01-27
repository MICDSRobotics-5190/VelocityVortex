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
import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.opencv.core.Mat;
import org.opencv.core.Size;

/*
 * This file is the linear Op-Mode made for the non-driver controlled
 * Autonomous period of an FTC match for Technoramic, Team 5190, in 2016-2017.
 *
 * The bot this was made for had an awesome west coast drivetrain, used motor encoders for once,
 * could use the camera to easily find the beacons and locate itself on the field, and
 * hit the button to score tons of points and release more balls into the field.
 * The motor encoders helped a lot in precise movement and getting on the ramp.
 */

@Autonomous(name="Blue + Beacon", group="Beacon OpMode")  // @Autonomous(...) is the other common choice
//@Disabled
public class BlueTeamBeacon extends LinearVisionOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    private Robot dan = new Robot();

    /*Declaring constant values */
    final int MOTOR_PULSE_PER_REVOLUTION = 7;
    final int MOTOR_GEAR_RATIO = 80;
    final int FULL_REVOLUTION = 1200;
    final int FLOOR_BLOCK = 2292;
    //final int FLOOR_TILE = x;
    //final int QUARTER_TURN = x;

    //Parts of the autonomous program
    private int step = 1;
    private boolean encodersInPosition;

    //Frames for OpenCV (Immediate Setup for OpenCV)
    int frameCount = 0;


    @Override
    public void runOpMode() throws InterruptedException {

        /* Initialize the hardware variables. The strings must
        correspond to the names in the configuration file. */
        dan.setupHardware(hardwareMap);

        //Prepare the encoders to be used
        dan.resetEncoders();

        boolean blueLeft = false;
        boolean redLeft = false;
        boolean blueRight = false;
        boolean redRight = false;

        //OpenCV Vision Setup
        waitForVisionStart();

        /**
         * Set the camera used for detection
         * PRIMARY = Front-facing, larger camera
         * SECONDARY = Screen-facing, "selfie" camera :D
         **/
        this.setCamera(Cameras.PRIMARY);

        /**
         * Set the frame size
         * Larger = sometimes more accurate, but also much slower
         * After this method runs, it will set the "width" and "height" of the frame
         **/
        this.setFrameSize(new Size(900, 900));

        /**
         * Enable extensions. Use what you need.
         * If you turn on the BEACON extension, it's best to turn on ROTATION too.
         */
        enableExtension(Extensions.BEACON);         //Beacon detection
        enableExtension(Extensions.ROTATION);       //Automatic screen rotation correction
        //enableExtension(Extensions.CAMERA_CONTROL); //Manual camera control

        /**
         * Set the beacon analysis method
         * Try them all and see what works!
         */
        beacon.setAnalysisMethod(Beacon.AnalysisMethod.FAST);


        /**
         * Set color tolerances
         * 0 is default, -1 is minimum and 1 is maximum tolerance
         */
        beacon.setColorToleranceRed(0);
        beacon.setColorToleranceBlue(0);

        /**
         * Set analysis boundary
         * You should comment this to use the entire screen and uncomment only if
         * you want faster analysis at the cost of not using the entire frame.
         * This is also particularly useful if you know approximately where the beacon is
         * as this will eliminate parts of the frame which may cause problems
         * This will not work on some methods, such as COMPLEX
         **/
        //beacon.setAnalysisBounds(new Rectangle(new Point(width / 2, height / 2), width - 200, 200));

        /**
         * Set the rotation parameters of the screen
         * If colors are being flipped or output appears consistently incorrect, try changing these.
         *
         * First, tell the extension whether you are using a secondary camera
         * (or in some devices, a front-facing camera that reverses some colors).
         *
         * It's a good idea to disable global auto rotate in Android settings. You can do this
         * by calling disableAutoRotate() or enableAutoRotate().
         *
         * It's also a good idea to force the phone into a specific orientation (or auto rotate) by
         * calling either setActivityOrientationAutoRotate() or setActivityOrientationFixed(). If
         * you don't, the camera reader may have problems reading the current orientation.
         */
        rotation.setIsUsingSecondaryCamera(false);
        rotation.disableAutoRotate();
        rotation.setActivityOrientationFixed(ScreenOrientation.LANDSCAPE);

        /**
         * Set camera control extension preferences
         *
         * Enabling manual settings will improve analysis rate and may lead to better results under
         * tested conditions. If the environment changes, expect to change these values.
         */
        cameraControl.setColorTemperature(CameraControlExtension.ColorTemperature.AUTO);
        cameraControl.setAutoExposureCompensation();

        telemetry.addData("Status", "Initialized!");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //dan.leftMotor.setMaxSpeed(MOTOR_PULSE_PER_REVOLUTION * MOTOR_GEAR_RATIO);
            //dan.rightMotor.setMaxSpeed(MOTOR_PULSE_PER_REVOLUTION * MOTOR_GEAR_RATIO);

            //Setting up some output for the user to see. (Usually for troubleshooting)
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Step", step);
            telemetry.addData("Right Motor Posn", dan.leftMotor.getCurrentPosition());
            telemetry.addData("Left Motor Posn", dan.rightMotor.getCurrentPosition());

            encodersInPosition = (dan.rightMotor.getCurrentPosition() >= dan.rightMotor.getTargetPosition() - 10
                    && dan.rightMotor.getCurrentPosition() <= dan.rightMotor.getTargetPosition() + 10);

            if(step == 1) {

                dan.leftMotor.setTargetPosition(FLOOR_BLOCK);
                dan.rightMotor.setTargetPosition(FLOOR_BLOCK);

                if (encodersInPosition) {

                    step = 2;

                    dan.resetEncoders();

                    dan.drivetrainPower(0);
                    sleep(500);

                } else {
                    dan.drivetrainPower(1);
                }

            } else if (step == 2) {

                dan.leftMotor.setTargetPosition(1 * FULL_REVOLUTION);
                dan.rightMotor.setTargetPosition(-1 * FULL_REVOLUTION);

                if (encodersInPosition) {

                    step = 3;

                    dan.resetEncoders();

                    dan.drivetrainPower(0);
                    sleep(500);

                } else {
                    dan.leftMotor.setPower(1);
                    dan.rightMotor.setPower(-1);
                }

            }  else if (step == 3){

                dan.leftMotor.setTargetPosition(FLOOR_BLOCK);
                dan.rightMotor.setTargetPosition(FLOOR_BLOCK);

                if (encodersInPosition) {

                    step = 4;

                    dan.resetEncoders();

                    dan.drivetrainPower(0);
                    sleep(500);

                } else {
                    dan.drivetrainPower(1);
                }

            } else if (step == 4) {

                dan.leftMotor.setTargetPosition(-1 * FULL_REVOLUTION);
                dan.rightMotor.setTargetPosition(1 * FULL_REVOLUTION);

                if (encodersInPosition) {

                    step = 6;

                    dan.resetEncoders();

                    dan.drivetrainPower(0);
                    sleep(500);

                } else {
                    dan.leftMotor.setPower(-1);
                    dan.rightMotor.setPower(1);
                }

            } else if (step == 5) {

                dan.leftMotor.setTargetPosition(3 * FLOOR_BLOCK);
                dan.rightMotor.setTargetPosition(3 * FLOOR_BLOCK);

                if (encodersInPosition) {

                    step = 6;

                    dan.resetEncoders();

                    dan.drivetrainPower(0);
                    sleep(500);

                } else {
                    dan.drivetrainPower(1);
                }

            } else if (step == 6) {

                dan.leftMotor.setTargetPosition(1 * FULL_REVOLUTION);
                dan.rightMotor.setTargetPosition(-1 * FULL_REVOLUTION);

                if (encodersInPosition) {

                    step = 7;

                    dan.resetEncoders();

                    dan.drivetrainPower(0);
                    sleep(500);

                } else {
                    dan.leftMotor.setPower(1);
                    dan.rightMotor.setPower(-1);
                }

                step = 7;

            } else if (step == 7){

                //Check Beacons
                telemetry.addData("Beacon Color", beacon.getAnalysis().getColorString());
                telemetry.addData("Beacon Center", beacon.getAnalysis().getLocationString());
                telemetry.addData("Beacon Confidence", beacon.getAnalysis().getConfidenceString());
                telemetry.addData("Beacon Buttons", beacon.getAnalysis().getButtonString());
                telemetry.addData("Screen Rotation", rotation.getScreenOrientationActual());
                telemetry.addData("Frame Rate", fps.getFPSString() + " FPS");
                telemetry.addData("Frame Size", "Width: " + width + " Height: " + height);
                telemetry.addData("Frame Counter", frameCount);

                //You can access the most recent frame data and modify it here using getFrameRgba() or getFrameGray()
                //Vision will run asynchronously (parallel) to any user code so your programs won't hang
                //You can use hasNewFrame() to test whether vision processed a new frame
                //Once you copy the frame, discard it immediately with discardFrame()
                if (hasNewFrame()) {
                    //Get the frame
                    Mat rgba = getFrameRgba();
                    Mat gray = getFrameGray();

                    //Discard the current frame to allow for the next one to render
                    discardFrame();

                    //Do all of your custom frame processing here
                    //For this demo, let's just add to a frame counter
                    frameCount++;
                }

                blueLeft = beacon.getAnalysis().isLeftBlue();
                redLeft = beacon.getAnalysis().isLeftBlue();
                blueRight = beacon.getAnalysis().isRightBlue();
                redRight = beacon.getAnalysis().isRightRed();

                sleep(500);

                step = 8;

            } else if (step == 8){

                

            }

            telemetry.update();

            //idle(); // OpenCV broke idle, we could troubleshoot later. Basically check LinearOpMode and LinearVisionOpmode.
        }
    }

    //Necessary for using Vuforia and outputting location matrices.
    String format(OpenGLMatrix transformationMatrix) {
        return transformationMatrix.formatAsTransform();
    }

}
