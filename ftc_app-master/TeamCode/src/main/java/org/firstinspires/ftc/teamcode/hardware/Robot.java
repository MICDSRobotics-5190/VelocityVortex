package org.firstinspires.ftc.teamcode.hardware;

/**
 * Created by BAbel on 4/10/2017.
 */

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {

    private Drivetrain drivetrain;
    private DcMotor launcher;
    private CRServo slider;

    public Robot(){
        drivetrain = null;
        launcher = null;
        slider = null;
    }

    public Robot(HardwareMap hardwareMap){
        drivetrain = new Drivetrain(hardwareMap);
        launcher = hardwareMap.dcMotor.get("launcher");
        slider = hardwareMap.crservo.get("slider");

        drivetrain.getLeftMotors().setDirections(DcMotorSimple.Direction.REVERSE);
        drivetrain.getRightMotors().setDirections(DcMotorSimple.Direction.FORWARD);
        drivetrain.setModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void stopMoving(){
        drivetrain.setPower(0);
        launcher.setPower(0);
        slider.setPower(0);
    }

    public Drivetrain getDrivetrain() {
        return drivetrain;
    }

    public DcMotor getLauncher() {
        return launcher;
    }

    public CRServo getSlider() {
        return slider;
    }

    public void setDrivetrain(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    public void setLauncher(DcMotor launcher) {
        this.launcher = launcher;
    }

    public void setSlider(CRServo slider) {
        this.slider = slider;
    }
}
