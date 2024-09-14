package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Subsystem;

@Config
public class DroneLauncher extends Subsystem {

    /** Hardware/Constants */
    private Servo stopper;

    public static double releasePos = 1, lockPos = 0.2;

    public DroneLauncher(HardwareMap hardwareMap) {
        stopper = hardwareMap.get(Servo.class, "droneLauncherStopper");
    }

    /** Base Functions */
    @Override
    public void init() {
        lock();
    }

    @Override
    public void stop() {

    }

    /** Mechanism Functions */
    public void release() {
        stopper.setPosition(releasePos);
    }

    public void lock() {
        stopper.setPosition(lockPos);
    }
}
