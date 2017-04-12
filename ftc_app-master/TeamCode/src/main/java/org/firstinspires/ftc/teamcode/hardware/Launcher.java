package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by BAbel on 4/11/2017.
 */

public class Launcher {

    private DcMotor launcherMotor;

    private double gearRatio;

    public Launcher(){
        launcherMotor = null;
        gearRatio = 0.666667;
    }

    public Launcher(HardwareMap hardwareMap){
        launcherMotor = hardwareMap.dcMotor.get("launcher");
        launcherMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        gearRatio = 0.666667;
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

        launcherMotor.setTargetPosition((int)gearRatio * Drivetrain.GEAR_BOX_RATIO * Drivetrain.PULSES_PER_ROTATION);
        launcherMotor.setPower(1);
    }
}
