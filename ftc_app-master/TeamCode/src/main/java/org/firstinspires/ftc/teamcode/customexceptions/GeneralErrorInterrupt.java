package org.firstinspires.ftc.teamcode.customexceptions;

/**
 * Created by amigala on 2/16/2017.
 * This Error is used for general purposes
 */

public class GeneralErrorInterrupt extends Exception {
    public GeneralErrorInterrupt() {

    }

    public GeneralErrorInterrupt(String message) {
        super(message);
    }

    /*
    public GeneralErrorInterrupt(Throwable cause) {
        super(cause);
    }
    public GeneralErrorInterrupt(String message, Throwable cause) {
        super(message, cause);
    }

    public GeneralErrorInterrupt(String message, Throwable cause, boolean enableSuppression, boolean writableStackTrace) {
        super(message, cause, enableSuppression, writableStackTrace);
    }*/
}