package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.HardwareMap;

public abstract class Subsystem {
    /**
     * Set hardware for init mode
     */
    public abstract void init();

    /**
     * Set hardware for stop mode
     */
    public abstract void stop();
}


/**
 * init
 * stop
 */