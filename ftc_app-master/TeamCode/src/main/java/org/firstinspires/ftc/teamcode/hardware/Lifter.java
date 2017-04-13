package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by amigala on 4/12/2017.
 */

public class Lifter {

    private DcMotor lifterMotor;

    /**
     * Constructor of the Lifter for the Robot Class
     * @param hardwareMap map given from OpMode
     */
    public Lifter(HardwareMap hardwareMap) {
        lifterMotor = hardwareMap.dcMotor.get("lifter");
        lifterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * This method returns the lifter motor object
     * @return the lifter motor object
     */
    public DcMotor getLifterMotor() {
        return lifterMotor;
    }

    /**
     * This will ascend the robot
     */
    public void ascend() {
        lifterMotor.setPower(1);
    }

    /**
     * This will descend the robot
     */
    public void descend() {
        lifterMotor.setPower(-1);
    }

    /**
     * This will stop the lifter motor
     */
    public void stop() {
        lifterMotor.setPower(0);
    }
}
