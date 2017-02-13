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

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.nio.ByteBuffer;


/*
 * This file is the iterative (non-linear) Op-Mode made for the driver controlled
 * TeleOp period of an FTC match for Technoramic, Team 5190, in 2016-2017.
 *
 * The bot this was made for had an awesome west coast drivetrain, used motor encoders for once,
 * <insert cool stuff about finished bot>, yep
 */

@Autonomous(name="Playback Blue Team", group="Playback")  // @Autonomous(...) is the other common choice
//@Disabled
public class PlaybackBlue extends LinearOpMode
{
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    private Robot dan = new Robot();


    String fileName = "BlueRecordedInputs";
    String filePath = "Paths";
    File file = null;
    FileInputStream in = null;

    byte[] input = new byte[480000];

    byte[][] recordedLeftStickArray = new byte[30000][4];
    byte[][] recordedRightStickArray = new byte[30000][4];
    byte[][] recordedRuntimeArray = new byte[30000][8];

    float[] leftStickData = new float[30000];
    float[] rightStickData = new float[30000];
    double[] runtimeData = new double[30000];

    int index = 0;

    /* Code to run ONCE when the driver hits INIT */
    @Override
    public void runOpMode() throws InterruptedException {

        /* Initialize the hardware variables. The strings must
        correspond to the names in the configuration file. */
        dan.setupHardware(hardwareMap);

        try {
            if(isExternalStorageReadable()) {

               File file = new File(FtcRobotControllerActivity.getAppContext().getFilesDir(), fileName);

                in = new FileInputStream(file);

                try {
                    in.read(input);
                } catch (IOException dataError){
                    telemetry.addData("Error", "Can't read into an array");
                }
            } else {
                telemetry.addData("Error", "Can't read any files");
            }
        } catch (FileNotFoundException issue){
            telemetry.addData("Error", "Can't read file");
        }

        for(int i = 0; i < recordedLeftStickArray.length; i++){
            for(int j = 0; j < recordedLeftStickArray[i].length; j++){
                recordedLeftStickArray[i][j] = input[(16 * i) + j];
            }
            for(int j = 0; j < recordedRightStickArray[i].length; j++){
                recordedRightStickArray[i][j] = input[(16 * i) + recordedLeftStickArray[i].length + j];
            }
            for(int j = 0; j < recordedRuntimeArray[i].length; j++){
                recordedRuntimeArray[i][j] = input[(16 * i) + recordedLeftStickArray[i].length + recordedRightStickArray[i].length + j];
            }
        }

        for(int i = 0; i < recordedLeftStickArray.length; i++){
            leftStickData[i] = byteArrayToFloat(recordedLeftStickArray[i]);
            rightStickData[i] = byteArrayToFloat(recordedRightStickArray[i]);
            runtimeData[i] = byteArrayToDouble(recordedRuntimeArray[i]);
        }

        try { in.close(); }
        catch (IOException issue){ telemetry.addData("Error", "Couldn't close input"); }

        telemetry.addData("Status", "Initialized");

        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("Status", "Running: " + runtime.toString());
            telemetry.addData("Right Motor", dan.rightMotor.getPower());
            telemetry.addData("Left Motor", dan.leftMotor.getPower());
            telemetry.addData("Spinner", dan.spinner.getPower());
            telemetry.addData("Flywheel", dan.flywheel.getPower());
            telemetry.addData("Beacon Hitter", dan.beaconSlider.getPower());

            double currentRuntime = getRuntime();

            while(currentRuntime > runtimeData[index]){
                index++;
            }

            dan.leftMotor.setPower(leftStickData[index]);
            dan.rightMotor.setPower(rightStickData[index]);

            telemetry.update();

        }

        dan.stopMoving();

        telemetry.addData("Status", "Successfully played back inputs!");
        telemetry.update();

    }

    public boolean isExternalStorageReadable() {
        String state = Environment.getExternalStorageState();
        if (Environment.MEDIA_MOUNTED.equals(state) ||
                Environment.MEDIA_MOUNTED_READ_ONLY.equals(state)) {
            return true;
        }
        return false;
    }

    public static float byteArrayToFloat(byte[] bytes){

        float f = ByteBuffer.wrap(bytes).getFloat();

        return f;

    }

    public static double byteArrayToDouble(byte[] bytes) {

        double d = ByteBuffer.wrap(bytes).getDouble();

        return d;
    }


}
