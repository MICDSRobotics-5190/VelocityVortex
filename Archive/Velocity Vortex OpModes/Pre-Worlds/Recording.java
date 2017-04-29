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
import android.provider.MediaStore;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import java.io.BufferedReader;
import java.io.DataInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStreamReader;

import java.lang.*;
import java.nio.ByteBuffer;


/*
 * This file is the iterative (non-linear) Op-Mode made for the driver controlled
 * TeleOp period of an FTC match for Technoramic, Team 5190, in 2016-2017.
 *
 * The bot this was made for had an awesome west coast drivetrain, used motor encoders for once,
 * <insert cool stuff about finished bot>, yep
 */

@TeleOp(name="Record Inputs", group="Recording")  // @Autonomous(...) is the other common choice
//@Disabled
public class Recording extends OpMode
{
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    private Robot dan = new Robot();


    private String fileName = "BlueRecordedInputs";
    private String filePath = "Paths";
    private File file = null;
    private FileOutputStream out = null;

    byte[] output = new byte[480000];

    /* Code to run ONCE when the driver hits INIT */
    @Override
    public void init() {

        /* Initialize the hardware variables. The strings must
        correspond to the names in the configuration file. */

        dan.setupHardware(hardwareMap);

        try {
            if(isExternalStorageWritable()) {

                File file = new File(FtcRobotControllerActivity.getAppContext().getFilesDir(), fileName);

                try {
                    file.createNewFile();
                } catch (IOException problem){
                    telemetry.addData("Error", "Can't create file");
                }

                out = new FileOutputStream(file);
                telemetry.addData("File", "Made!");
            } else {
                telemetry.addData("Error", "Can't write to any files");
            }
        } catch (FileNotFoundException issue){
            telemetry.addData("Error", "Can't make file");
        }

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


        if(gamepad1.left_bumper){
            dan.spinner.setPower(1);
        } else if (gamepad1.right_bumper){
            dan.spinner.setPower(-1);
        } else {
            dan.spinner.setPower(0);
        }

        if(gamepad1.a){
            dan.flywheel.setPower(1);
        } else if(gamepad1.b) {
            dan.flywheel.setPower(-1);
        } else {
            dan.flywheel.setPower(0);
        }

        if(gamepad1.dpad_left){
            dan.beaconSlider.setPower(1);
        } else if (gamepad1.dpad_right){
            dan.beaconSlider.setPower(-1);
        } else {
            dan.beaconSlider.setPower(0);
        }

        if(gamepad1.start){
            dan.stopMoving();
        }

        byte[] leftStick = floatToByteArray(-gamepad1.left_stick_y);
        byte[] rightStick = floatToByteArray(-gamepad1.right_stick_y);

        dan.leftMotor.setPower(-gamepad1.left_stick_y);
        dan.rightMotor.setPower(-gamepad1.right_stick_y);

        byte[] runtime = doubleToByteArray(getRuntime());

        for (int index = 0; index < output.length; index += 16){

            for(int i = index; i < index + 4 ; i++) {
                output[i] = leftStick[i - index];
            }

            for(int i = index + 4; i < index + 8 ; i++) {
                output[i] = rightStick[i - (index + 4)];
            }

            for(int i = index + 8; i < index + 16 ; i++) {
                output[i] = runtime[i - (index + 8)];
            }
        }

    }


    /* Code to run ONCE after the driver hits STOP */
    @Override
    public void stop() {
        dan.stopMoving();

        try {
            out.write(output);
            out.close();
        } catch (IOException issue){
            telemetry.addData("Error", "Couldn't write to RecordedInputs");
        }
        telemetry.addData("Status", "Successfully recorded inputs!");
        telemetry.update();

    }

    public boolean isExternalStorageWritable() {
        String state = Environment.getExternalStorageState();
        if (Environment.MEDIA_MOUNTED.equals(state)) {
            return true;
        }
        return false;
    }

    public static byte[] floatToByteArray(float value){

        int bits = Float.floatToIntBits(value);
        byte[] bytes = new byte[4];
        bytes[0] = (byte)(bits & 0xff);
        bytes[1] = (byte)((bits >> 8) & 0xff);
        bytes[2] = (byte)((bits >> 16) & 0xff);
        bytes[3] = (byte)((bits >> 24) & 0xff);
        return bytes;

    }

    public static byte[] doubleToByteArray(double value) {
        byte[] bytes = new byte[8];
        ByteBuffer.wrap(bytes).putDouble(value);
        return bytes;
    }


}
