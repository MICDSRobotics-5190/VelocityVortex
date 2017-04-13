package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by BAbel on 4/11/2017.
 */

public class Launcher implements EncoderValues {

    private DcMotor launcherMotor;

    private double gearRatio;

    public Launcher(){
        launcherMotor = null;
    }

    public Launcher(HardwareMap hardwareMap){
        launcherMotor = hardwareMap.dcMotor.get("launcher");
        launcherMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public DcMotor getLauncherMotor() {
        return launcherMotor;
    }

    public void setLauncherMotor(DcMotor launcherMotor) {
        this.launcherMotor = launcherMotor;
    }

    public void fullRotation(){
        launcherMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcherMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        launcherMotor.setTargetPosition(FULL_ROTATION);
        launcherMotor.setPower(1);
    }
}