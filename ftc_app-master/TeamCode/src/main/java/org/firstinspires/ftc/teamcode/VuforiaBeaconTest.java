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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.vuforia.Frame;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CloseableFrame;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

import java.lang.Object;
import java.nio.ByteBuffer;
import java.nio.ByteBuffer.*;
import java.nio.Buffer;

//import org.lasarobotics.vision.opmode.LinearVisionOpMode;

/*
 * This file is the linear Op-Mode made for the non-driver controlled
 * Autonomous period of an FTC match for Technoramic, Team 5190, in 2016-2017.
 *
 * The bot this was made for had an awesome west coast drivetrain, used motor encoders for once,
 * could use the camera to easily find the beacons and locate itself on the field, and
 * hit the button to score tons of points and release more balls into the field.
 * The motor encoders helped a lot in precise movement and getting on the ramp.
 */

@Autonomous(name="Vuforia + Beacon Test", group="Cool")  // @Autonomous(...) is the other common choice
//@Disabled
public class VuforiaBeaconTest extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    public static final String TAG = "Vuforia Sample";

    OpenGLMatrix lastLocation = null;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

    Beacon beacon;

    //Frames for OpenCV (Immediate Setup for OpenCV)
    int frameCount = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        //Vuforia Setup (OpenCV Setup?)
        /**
         * Start up Vuforia, telling it the id of the view that we wish to use as the parent for
         * the camera monitor feedback. We chose the back camera, however the front could
         * be more convenient too. Easy change.
         *
         * Note that in addition to indicating which camera is in use, we also need to tell the system
         * the location of the phone on the robot; see phoneLocationOnRobot below.
         *
         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below
         * is a key that Blake got to use, so we're all set. In the future, it may not work
         * after he leaves, in which case you will need to go to https://developer.vuforia.com/license-manager
         * to get one again.
         *
         * Valid Vuforia license keys are  380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy it from the Vuforia web site and paste it in to
         * your code as the value of the 'vuforiaLicenseKey'.
         */

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AXmkGNH/////AAAAGYQoI5Oxd0xYgQIS+WWvhqRgDbmxAw8+4qC9TSBr3OQpP8oUs2lxaU4x2ptI6ldk8kJ9QfgFGu6/K6Xtm4jbx27/AE6gIgzDSEoqgJ9TFEqwJCdywmzroqzWu97Tfh8Zp14gI9Y0gH59SqMBwlTDbR5sM4XkMQobfSTmP8LjFnuplw/lqlcZGGlfZ8WJTdFdIJmUkb1S6L5cPQosxgIm5goCPXWNc8WPhUqV+DLOovlU50ecwSLZrqLyIBdEGmKwB2Au8nZTT7+fO/I9ouKpzy0hwtAbthgidqC2GL+sxt13JnzXXR9d7hT65UIvtZqbobq96pkOpBLV/ztrLh9ayCF2i2vv9wiOHkZPbSgtbyFP";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data sets that for the trackable objects we wish to track. These particular data
         * sets are stored in the 'assets' part of our application (you'll see them in the Android
         * Studio 'Project' view over there on the left of the screen). You can make your own datasets
         * with the Vuforia Target Manager: https://developer.vuforia.com/target-manager. PDFs for the
         * example "StonesAndChips", datasets can be found in in this project in the
         * documentation directory.
         */

        VuforiaTrackables velocityVortexTargets = this.vuforia.loadTrackablesFromAsset("VelocityVortexTargets");
        VuforiaTrackable wheels = velocityVortexTargets.get(0);
        wheels.setName("Wheels");  // Wheels, Blue side near Ramp

        VuforiaTrackable tools = velocityVortexTargets.get(1);
        tools.setName("Tools");  // Tools, Red side away from Ramp

        VuforiaTrackable legos = velocityVortexTargets.get(2);
        legos.setName("Legos");  // Legos, Blue side away from Ramp

        VuforiaTrackable gears = velocityVortexTargets.get(3);
        gears.setName("Gears");  // Gears, Red side near Ramp


        /** For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(velocityVortexTargets);

        /*
         * We use units of mm here because that's the recommended units of measurement for the
         * size values specified in the XML for the ImageTarget trackables in data sets. E.g.:
         *      <ImageTarget name="stones" size="247 173"/>
         * You don't *have to* use mm here, but the units here and the units used in the XML
         * target configuration files *must* correspond for the math to work out correctly.
         */

        float mmPerInch        = 25.4f;
        float mmBotWidth       = 18 * mmPerInch;            // ... or whatever is right for your robot
        float mmFTCFieldWidth  = (12*12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels

        /**
         * In order for localization to work, we need to tell the system where each target we
         * wish to use for navigation resides on the field, and we need to specify where on the robot
         * the phone resides. These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * For the most part, you don't need to understand the details of the math of how transformation
         * matrices work inside (as fascinating as that is, truly). Just remember these key points:
         * <ol>
         *
         *     <li>You can put two transformations together to produce a third that combines the effect of
         *     both of them. If, for example, you have a rotation transform R and a translation transform T,
         *     then the combined transformation matrix RT which does the rotation first and then the translation
         *     is given by {@code RT = T.multiplied(R)}. That is, the transforms are multiplied in the
         *     <em>reverse</em> of the chronological order in which they applied.</li>
         *
         *     <li>A common way to create useful transforms is to use methods in the {@link OpenGLMatrix}
         *     class and the Orientation class. See, for example, {@link OpenGLMatrix#translation(float,
         *     float, float)}, {@link OpenGLMatrix#rotation(AngleUnit, float, float, float, float)}, and
         *     {@link Orientation#getRotationMatrix(AxesReference, AxesOrder, AngleUnit, float, float, float)}.
         *     Related methods in {@link OpenGLMatrix}, such as {@link OpenGLMatrix#rotated(AngleUnit,
         *     float, float, float, float)}, are syntactic shorthands for creating a new transform and
         *     then immediately multiplying the receiver by it, which can be convenient at times.</li>
         *
         *     <li>If you want to break open the black box of a transformation matrix to understand
         *     what it's doing inside, use {@link MatrixF#getTranslation()} to fetch how much the
         *     transform will move you in x, y, and z, and use {@link Orientation#getOrientation(MatrixF,
         *     AxesReference, AxesOrder, AngleUnit)} to determine the rotational motion that the transform
         *     will impart. See {@link #format(OpenGLMatrix)} below for an example.</li>
         *
         * </ol>
         *
         * We place the vision targets underneath their own respective beacons, rotated and arranged
         * according to the field setup guide online. More info to follow.
         *
         * See the doc folder of this project for a description of the field Axis conventions.
         *
         * Initially the target is conceptually lying at the origin of the field's coordinate system
         * (the center of the field), facing up.
         *
         * In this configuration, the target's coordinate system aligns with that of the field.
         *
         * ll of the Z axis translations are because th etargets are 1.5 inches off the floor tiles.
         *
         * To place the Stones Target on the Red Audience wall:
         * - First we rotate it 90 around the field's X axis to flip it upright
         * - Then we rotate it  90 around the field's Z access to face it away from the audience.
         * - Finally, we translate it back along the X axis towards the red audience wall.
         * - We also push it a little backwards or forwards to match the actual beacon's position.
         * The 12 or 36 are the numbers we added, the FTCFieldWidth was for putting it along the walls.
         */

        OpenGLMatrix gearsLocationOnField = OpenGLMatrix
                /* We translate the target on the Red audience wall and along it under the beacon.
                * (negative x, negative y)*/
                .translation(-mmFTCFieldWidth/2, -12*mmPerInch, (float)(1.5*mmPerInch))
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        gears.setLocation(gearsLocationOnField);
        RobotLog.ii(TAG, "Gears=%s", format(gearsLocationOnField));

        OpenGLMatrix toolsLocationOnField = OpenGLMatrix
                /* We translate the target on the Red audience wall and along it under the beacon.
                * (negative x, positive y)*/
                .translation(-mmFTCFieldWidth/2, 36*mmPerInch, (float)(1.5*mmPerInch))
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        tools.setLocation(toolsLocationOnField);
        RobotLog.ii(TAG, "Tools=%s", format(toolsLocationOnField));

       /*
        * To place the Stones Target on the Blue Audience wall:
        * - First we rotate it 90 around the field's X axis to flip it upright
        * - Finally, we translate it along the Y axis towards the blue audience wall.
        */
        OpenGLMatrix wheelsLocationOnField = OpenGLMatrix
                /* We translate the target on the Blue audience wall and along it under the beacon.
                * Positive Y, positive X*/
                .translation(12*mmPerInch, mmFTCFieldWidth/2, (float)(1.5*mmPerInch))
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        wheels.setLocation(wheelsLocationOnField);
        RobotLog.ii(TAG, "Wheels=%s", format(wheelsLocationOnField));

        OpenGLMatrix legosLocationOnField = OpenGLMatrix
                /* We translate the target on the Blue audience wall and along it under the beacon.
                * Positive Y, negative X*/
                .translation(-36*mmPerInch, mmFTCFieldWidth/2, (float)(1.5*mmPerInch))
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        legos.setLocation(legosLocationOnField);
        RobotLog.ii(TAG, "Legos=%s", format(wheelsLocationOnField));

        /**
         * Create a transformation matrix describing where the phone is on the robot. Here, we
         * put the phone on the right hand side of the robot with the screen facing in (see our
         * choice of BACK camera above) and in landscape mode. Starting from alignment between the
         * robot's and phone's axes, this is a rotation of -90deg along the Y axis.
         *
         * When determining whether a rotation is positive or negative, consider yourself as looking
         * down the (positive) axis of rotation from the positive towards the origin. Positive rotations
         * are then CCW, and negative rotations CW. An example: consider looking down the positive Z
         * axis towards the origin. A positive rotation about Z (ie: a rotation parallel to the the X-Y
         * plane) is then CCW, as one would normally expect from the usual classic 2D geometry.
         */
        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(0, 6 * mmPerInch, 0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.YZY,
                        AngleUnit.DEGREES, -90, 0, 0));
        RobotLog.ii(TAG, "phone=%s", format(phoneLocationOnRobot));

        /**
         * Let the trackable listeners we care about know where the phone is. We know that each
         * listener is a {@link VuforiaTrackableDefaultListener} and can so safely cast because
         * we have not ourselves installed a listener of a different type.
         */
        ((VuforiaTrackableDefaultListener)wheels.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)tools.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)legos.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)gears.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);

        Image rgb = null;
        int count = 0;
        long numImages;
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB888, true); //This line is very important, make sure the keep the format constant throughout the program. I'm using the MotoG2. I've also tested on the ZTE speeds and I found that they use RGB888
        this.vuforia.setFrameQueueCapacity(1); //tells VuforiaLocalizer to only store one frame at a time

        telemetry.addData("Status", "Initialized!");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        runtime.reset();

        velocityVortexTargets.activate();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            for (VuforiaTrackable trackable : allTrackables) {
                /**
                 * getUpdatedRobotLocation() will return null if no new information is available since
                 * the last time that call was made, or if the trackable is not currently visible.
                 * getRobotLocation() will return null if the trackable is not currently visible.
                 */
                telemetry.addData(trackable.getName(), ((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible() ? "Visible" : "Not Visible");    //

                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
            }
            /**
             * Provide feedback as to where the robot was last located (if we know).
             */
            if (lastLocation != null) {
                //  RobotLog.vv(TAG, "robot=%s", format(lastLocation));
                telemetry.addData("Pos", format(lastLocation));
            } else {
                telemetry.addData("Pos", "Unknown");
            }

            telemetry.addData("Status", "Run Time: " + runtime.toString());

            CloseableFrame rawFrame = vuforia.getFrameQueue().take();

            numImages = rawFrame.getNumImages();
            for (int i = 0; i < numImages; i++) { //finds a frame that is in color, not grayscale
                if (rawFrame.getImage(i).getFormat() == PIXEL_FORMAT.RGB888) {
                    rgb = rawFrame.getImage(i);
                    break;
                }
            }

            if(rgb != null){
                ByteBuffer pixelData = ByteBuffer.allocate(5000000);

                pixelData.put(rgb.getPixels().duplicate());

                byte[] pixelArray = pixelData.array();

                telemetry.addData("Status", "Before Mat putting");
                telemetry.update();
                sleep(3000);

                Mat colorPicture = new Mat();
                Mat grayPicture = new Mat();

                colorPicture.put(rgb.getHeight(), rgb.getWidth(), pixelArray);

                telemetry.addData("Status", "Before converting");
                telemetry.update();
                sleep(3000);

                //Imgproc.cvtColor(colorPicture, grayPicture, Imgproc.COLOR_RGB2GRAY);

                telemetry.addData("Channels", colorPicture.channels());
                telemetry.addData("Width", colorPicture.width());
                telemetry.addData("Height", colorPicture.height());
                telemetry.addData("Depth", colorPicture.depth());
                telemetry.update();
                sleep(10000);

                telemetry.addData("Status", "Before frameanalysis");
                telemetry.update();
                sleep(3000);

                Beacon.BeaconAnalysis analysis = beacon.analyzeFrame(colorPicture, grayPicture);

                telemetry.addData("Left", analysis.getStateLeft());
                telemetry.addData("Right", analysis.getStateRight());
                telemetry.update();
            }

            idle(); // OpenCV broke idle, we could troubleshoot later. Basically check LinearOpMode and LinearVisionOpmode.
        }
    }

    //Necessary for using Vuforia and outputting location matrixes.
    String format(OpenGLMatrix transformationMatrix) {
        return transformationMatrix.formatAsTransform();
    }

}
