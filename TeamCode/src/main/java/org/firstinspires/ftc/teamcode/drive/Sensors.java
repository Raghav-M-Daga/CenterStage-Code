package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.Subsystem;

@Config
public class Sensors extends Subsystem {

    /** Hardware/Constants */
//    private RevBlinkinLedDriver ledDriver;
//    private RevBlinkinLedDriver.BlinkinPattern pattern;
    private DistanceSensor frontDistanceSensor;

    private  DistanceSensor backDistanceSensor;
    private Rev2mDistanceSensor sensorTimeOfFlight;
//    private Rev2mDistanceSensor ultrasonicDist;
    private DistanceSensor intakeFrontDistance;
    private ColorSensor intakeFrontColor;
    private DistanceSensor intakeBackDistance;
    private ColorSensor intakeBackColor;


//    private double white = 0.0, purple = 0.0, yellow = 0.0, green = 0.0;

    public static double intakeFrontDistanceThresh = 3, intakeBackDistanceThresh = 3;

//    public enum PIXEL_COLOR {
//        WHITE,
//        PURPLE,
//        YELLOW,
//        GREEN,
//        UNKNOWN
//    }

    public Sensors(HardwareMap hardwareMap) {
        // 2M facing the front of the robot
        frontDistanceSensor = hardwareMap.get(DistanceSensor.class, "frontDistanceSensor");
        backDistanceSensor = hardwareMap.get(DistanceSensor.class, "backDistanceSensor");
        // Color sensor looking at the front pixel on conveyor
        intakeFrontDistance = hardwareMap.get(DistanceSensor.class, "intakeFront");
        intakeFrontColor = hardwareMap.get(ColorSensor.class, "intakeFront");
        // Color sensor looking at the back pixel on the conveyor
        intakeBackDistance = hardwareMap.get(DistanceSensor.class, "intakeBack");
        intakeBackColor = hardwareMap.get(ColorSensor.class, "intakeBack");
//        ledDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
//        ultrasonicDist = hardwareMap.get(Rev2mDistanceSensor.class, "ultrasonicDist");

        sensorTimeOfFlight = (Rev2mDistanceSensor) frontDistanceSensor;
//        pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
    }

    /** Base Functions */
    @Override
    public void init() {
//        ledDriver.setPattern(pattern);
    }

    @Override
    public void stop() {

    }

    /** Sensor Functions */
    public boolean intakeFrontHasPixel() {
        return intakeFrontDistance.getDistance(DistanceUnit.CM) < intakeFrontDistanceThresh;
    }

    public boolean intakeBackHasPixel() {
        return intakeBackDistance.getDistance(DistanceUnit.CM) < intakeBackDistanceThresh;
    }

    public int pixelCount() {
        if (intakeBackDistance.getDistance(DistanceUnit.CM) < intakeBackDistanceThresh && !(intakeFrontDistance.getDistance(DistanceUnit.CM) < intakeFrontDistanceThresh)) {
            return 1;
        } else if (!(intakeBackDistance.getDistance(DistanceUnit.CM) < intakeBackDistanceThresh) && intakeFrontDistance.getDistance(DistanceUnit.CM) < intakeFrontDistanceThresh) {
            return 1;
        } else if (intakeFrontDistance.getDistance(DistanceUnit.CM) < intakeFrontDistanceThresh && intakeBackDistance.getDistance(DistanceUnit.CM) < intakeBackDistanceThresh) {
            return 2;
        }

        return 0;
    }

    public boolean fullPixelCount() {
        return (intakeFrontDistance.getDistance(DistanceUnit.CM) < intakeFrontDistanceThresh && intakeBackDistance.getDistance(DistanceUnit.CM) < intakeBackDistanceThresh);
    }

    public double getIntakeFrontDistance() {
        return intakeFrontDistance.getDistance(DistanceUnit.CM);
    }

    public double getIntakeBackDistance() {
        return intakeBackDistance.getDistance(DistanceUnit.CM);
    }

//    public double getUltrasonicDistance() {
//        return ultrasonicDist.getDistance(DistanceUnit.CM);
//    }

    public double getIntakeFrontAlpha() {
        return intakeFrontColor.alpha();
    }

    public double getIntakeFrontArgb() {
        return intakeFrontColor.argb();
    }

    public double getIntakeFrontRed() {
        return intakeFrontColor.red();
    }

    public double getIntakeFrontGreen() {
        return intakeFrontColor.green();
    }

//    public PIXEL_COLOR getIntakeFrontColor() {
//        PIXEL_COLOR color = PIXEL_COLOR.WHITE;
//
//        return color;
//    }
//
//    public PIXEL_COLOR getIntakeBackColor() {
//        PIXEL_COLOR color = PIXEL_COLOR.WHITE;
//
//        return color;
//    }

    public double getFrontDistance() {
        return frontDistanceSensor.getDistance(DistanceUnit.CM);
    }

    public double getBackDistance() {
        return backDistanceSensor.getDistance(DistanceUnit.CM);
    }

    public boolean frontDistanceTimedOut() {
        return sensorTimeOfFlight.didTimeoutOccur();
    }

//    public void setLEDPurple() {
//        ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
//    }
//
//    public void setLEDWhite() {
//        ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
//    }
//
//    public void setLED(String pattern) {
//        ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.valueOf(pattern));
//    }
}
