package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by amigala on 4/12/2017.
 */

public class Intake {

    private DcMotor intakeMotor;

    /**
     * Constructor of the Intake for the Robot class
     * @param hardwareMap map from OpMode
     */
    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.dcMotor.get("intake");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Gives Intake Object
     * @return returns intake object
     */
    public DcMotor getIntakeMotor() {
        return intakeMotor;
    }

    /**
     * Method for taking in the ball
     */
    public void takeInBall() {
        intakeMotor.setPower(1);
    }

    /**
     * Reverses the intake
     */
    public void purgeBall() {
        intakeMotor.setPower(-1);
    }

    /**
     * Stops the intake motor
     */
    public void stop() {
        intakeMotor.setPower(0);
    }
}