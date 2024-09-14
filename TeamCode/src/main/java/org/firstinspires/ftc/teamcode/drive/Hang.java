package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Subsystem;

@Config
public class Hang extends Subsystem {

    /** Hardware/Constants */
    private Servo left, right;

    public static double leftRaisePos = 1.0, rightRaisePos = 0.0;
    public static double leftLowerPos = 0.0, rightLowerPos = 1.0;

    public Hang(HardwareMap hardwareMap) {
        left = hardwareMap.get(Servo.class, "hangLeft");
        right = hardwareMap.get(Servo.class, "hangRight");
    }

    /** Base Functions */
    @Override
    public void init() {

    }

    @Override
    public void stop() {

    }

    /** Mechanism Functions */
    public void raise() {
        left.setPosition(leftRaisePos);
        right.setPosition(rightRaisePos);
    }

    public void lower() {
        left.setPosition(leftLowerPos);
        right.setPosition(rightLowerPos);
    }
}
