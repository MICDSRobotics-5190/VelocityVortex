package org.firstinspires.ftc.teamcode.hardware;

/**
 * Created by BAbel on 4/10/2017.
 */

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {

    private TankDrive tankDrive;
    private Launcher launcher;
    private CRServo slider;
    private Intake intake;
    private ColorSensor colorSensor;
    //private Lifter lifter;

    private float hsvValues[] = new float[3];
    private int rgbValues[] = new int[3];

    public Robot(){
        tankDrive = null;
        launcher = null;
        slider = null;
        intake = null;
        colorSensor = null;
        //lifter = null;
    }

    public Robot(HardwareMap hardwareMap){
        tankDrive = new TankDrive(hardwareMap);
        launcher = new Launcher(hardwareMap);
        slider = hardwareMap.crservo.get("slider");
        intake = new Intake(hardwareMap);
        //lifter = new Lifter(hardwareMap);
        colorSensor = hardwareMap.colorSensor.get("color");

        tankDrive.getLeftMotors().setDirections(DcMotorSimple.Direction.FORWARD);
        tankDrive.getRightMotors().setDirections(DcMotorSimple.Direction.REVERSE);
        tankDrive.setModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void stopMoving(){
        tankDrive.setPower(0);
        launcher.getLauncherMotor().setPower(0);
        slider.setPower(0);
        intake.getIntakeMotor().setPower(0);
        //lifter.getLifterMotor().setPower(0);
        colorSensor.enableLed(false);
    }

    public TankDrive getTankDrive() {
        return tankDrive;
    }

    public Launcher getLauncher() {
        return launcher;
    }

    public CRServo getSlider() {
        return slider;
    }

    public Intake getIntake() { return intake; }

    //public Lifter getLifter() { return lifter; }

    public void setDrivetrain(Drivetrain drivetrain) {
        this.tankDrive = tankDrive;
    }

    public void setLauncher(Launcher launcher) {
        this.launcher = launcher;
    }

    public void setSlider(CRServo slider) {
        this.slider = slider;
    }

    public ColorSensor getColorSensor() {
        return colorSensor;
    }

    public void setColorSensor(ColorSensor colorSensor) {
        this.colorSensor = colorSensor;
    }

    public void colorScan(){
        rgbValues[0] = colorSensor.red();
        rgbValues[1] = colorSensor.green();
        rgbValues[2] = colorSensor.blue();
        Color.RGBToHSV(rgbValues[0], rgbValues[1], rgbValues[2], hsvValues);
    }

    public float[] getHsvValues() {
        return hsvValues;
    }

    public int[] getRgbValues() {
        return rgbValues;
    }
}
