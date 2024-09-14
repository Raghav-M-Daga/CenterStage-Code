package org.firstinspires.ftc.teamcode.drive;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Subsystem;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

@Config
public class Intake extends Subsystem {

    /** Hardware/Constants */
    private DcMotorEx intake;
    private Servo rightRotate, leftRotate;
    private Sensors sensor;

    public static double collectPow = 0.8, spitOutPow = -1, alignPow = 0.4;
    public static double rotateRightUpPos = 0.62, rotateLeftUpPos = 0.3;
    public static double rotateRightDownPos = 0.08, rotateLeftDownPos = 0.84;
    public static double pixel5PosRight = 0.25; public static double pixel5PosLeft = 0.67;
    public static double pixel4PosRight = 0.20; public static double pixel4PosLeft = 0.72;
    public static double pixel3PosRight = 0.17; public static double pixel3PosLeft = 0.75;
    public static double pixel2PosRight = 0.12; public static double pixel2PosLeft = 0.80;
    public static double pixel1PosRight = 0.08; public static double pixel1PosLeft = 0.84;
    public static double rotateRightIntakeMovingPos = 0.5; public static double rotateLeftIntakeMovingPos = 0.42;

    public static double pursuitDistance = 5;
    public static double backUpDist = 19, backUpDist2 = 19;
    public static double closeUpDist = 15;
    public static double closeUpDist2 = 16.5;
    public static double closeUpDist3 = 15;
    public static double wiggleFront = 18;
    public static double intakePursuitPow = 0.2;
    public static double armDropTime = 0.35;
    public static double stackWaitTime = 0.3;

    public Intake(HardwareMap hardwareMap, Sensors sensor) {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        rightRotate = hardwareMap.get(Servo.class, "intakeRightRotate");
        leftRotate = hardwareMap.get(Servo.class, "intakeLeftRotate");

        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        this.sensor = sensor;
    }

    /** Base Functions */
    @Override
    public void init() {
        intake.setPower(0);
        rightRotate.setPosition(rotateRightUpPos);
        leftRotate.setPosition(rotateLeftUpPos);
    }

    @Override
    public void stop() {
        intake.setPower(0);
        rightRotate.setPosition(rotateRightUpPos);
        leftRotate.setPosition(rotateLeftUpPos);
    }

    /** Mechanism Functions */
    public void collectTopPixelsAuto(Babaji babaji) throws InterruptedException {
        babaji.setMotorPowers(0.0, 0.0, 0.0, 0.0);
        ElapsedTime timer = new ElapsedTime();
        setPow(collectPow);
        intakePixel5();
        timer.reset();
        while (!sensor.intakeBackHasPixel()) {
            if (timer.time(TimeUnit.SECONDS) > 2.5) {
                break;
            }
        }
        sleep(1000);
        intakePixel4();
        timer.reset();
        while (!sensor.intakeFrontHasPixel()) {
            if (timer.time(TimeUnit.SECONDS) > 2.5) {
                intakePixel3();
            } else if (timer.time(TimeUnit.SECONDS) > 5) {
                break;
            }
        }
        sleep(1000);
        setPow(0);
    }

    public void collectPixelsFarCycle(Babaji babaji) throws InterruptedException {
        babaji.setMotorPowers(0.0, 0.0, 0.0, 0.0);
        boolean pixelIn = false;
        ElapsedTime timer = new ElapsedTime();
        setPow(collectPow);
        raiseIntakeMoving();
//        sleep(300);
//        timer.reset();

        while (!sensor.intakeBackHasPixel()) {
            if (!pixelIn) {
                babaji.setMotorPowers(intakePursuitPow, intakePursuitPow, intakePursuitPow, intakePursuitPow);
                if (sensor.getFrontDistance() <= closeUpDist2){
                    babaji.setMotorPowers(0,0,0,0);
                    intakePixel4();
                    timer.reset();
                    pixelIn = true;
                    while (timer.time(TimeUnit.SECONDS) < stackWaitTime) {}
                    babaji.setMotorPowers(-intakePursuitPow, -intakePursuitPow, -intakePursuitPow, -intakePursuitPow);
                    timer.reset();
                }
            } else {
                if (sensor.getFrontDistance() > backUpDist){
                    intakePixel1();
                    while (timer.time(TimeUnit.SECONDS) < armDropTime){
                        babaji.setMotorPowers(0,0,0,0);
                    }
//                    babaji.setMotorPowers(-intakePursuitPow, -intakePursuitPow, -intakePursuitPow, -intakePursuitPow);
                } else {
                    timer.reset();
                }
            }
        }
        babaji.setMotorPowers(0, 0, 0, 0);
        intakePixel4();
        sleep(500);
        pixelIn = false;
        timer.reset();
        while (!sensor.intakeFrontHasPixel()) {
            if (!pixelIn) {
                babaji.setMotorPowers(intakePursuitPow, intakePursuitPow, intakePursuitPow, intakePursuitPow);
                if (sensor.getFrontDistance() <= closeUpDist2){
                    babaji.setMotorPowers(0,0,0,0);
                    intakePixel3();
                    timer.reset();
                    pixelIn = true;
                    while (timer.time(TimeUnit.SECONDS) < stackWaitTime) {}
                    babaji.setMotorPowers(-intakePursuitPow, -intakePursuitPow, -intakePursuitPow, -intakePursuitPow);
                    timer.reset();
                }
            } else {
                if (sensor.getFrontDistance() > backUpDist){
                    intakePixel1();
                    while (timer.time(TimeUnit.SECONDS) < armDropTime){
                        babaji.setMotorPowers(0,0,0,0);
                    }
//                    babaji.setMotorPowers(-intakePursuitPow, -intakePursuitPow, -intakePursuitPow, -intakePursuitPow);
                } else {
                    timer.reset();
                }
            }
        }
        babaji.setMotorPowers(0,0,0,0);
        sleep(1000);
        setPow(0);
    }

    public void collectPixelsFarStack(Babaji babaji) throws InterruptedException {
        babaji.setMotorPowers(0.0, 0.0, 0.0, 0.0);
        boolean pixelIn = false;
        ElapsedTime timer = new ElapsedTime();
        setPow(collectPow);
        raiseIntakeMoving();
//        sleep(300);
//        timer.reset();

        while (!sensor.intakeBackHasPixel()) {
            if (!pixelIn) {
                babaji.setMotorPowers(intakePursuitPow, intakePursuitPow, intakePursuitPow, intakePursuitPow);
                if (sensor.getFrontDistance() <= closeUpDist2){
                    babaji.setMotorPowers(0,0,0,0);
                    intakePixel5();
                    timer.reset();
                    pixelIn = true;
                    while (timer.time(TimeUnit.SECONDS) < stackWaitTime) {}
                    babaji.setMotorPowers(-intakePursuitPow, -intakePursuitPow, -intakePursuitPow, -intakePursuitPow);
                    timer.reset();
                }
            } else {
                if (sensor.getFrontDistance() > backUpDist){
                    intakePixel1();
                    while (timer.time(TimeUnit.SECONDS) < armDropTime){
                        babaji.setMotorPowers(0,0,0,0);
                    }
//                    babaji.setMotorPowers(-intakePursuitPow, -intakePursuitPow, -intakePursuitPow, -intakePursuitPow);
                } else {
                    timer.reset();
                }
            }
        }
        babaji.setMotorPowers(0, 0, 0, 0);
        intakePixel5();
        sleep(500);
        pixelIn = false;
        timer.reset();
        while (!sensor.intakeFrontHasPixel()) {
            if (!pixelIn) {
                babaji.setMotorPowers(intakePursuitPow, intakePursuitPow, intakePursuitPow, intakePursuitPow);
                if (sensor.getFrontDistance() <= closeUpDist2){
                    babaji.setMotorPowers(0,0,0,0);
                    intakePixel4();
                    timer.reset();
                    pixelIn = true;
                    while (timer.time(TimeUnit.SECONDS) < stackWaitTime) {}
                    babaji.setMotorPowers(-intakePursuitPow, -intakePursuitPow, -intakePursuitPow, -intakePursuitPow);
                    timer.reset();
                }
            } else {
                if (sensor.getFrontDistance() > backUpDist2){
                    intakePixel1();
                    while (timer.time(TimeUnit.SECONDS) < armDropTime){
                        babaji.setMotorPowers(0,0,0,0);
                    }
//                    babaji.setMotorPowers(-intakePursuitPow, -intakePursuitPow, -intakePursuitPow, -intakePursuitPow);
                } else {
                    timer.reset();
                }
            }
        }
        babaji.setMotorPowers(0,0,0,0);
//        sleep(1000);
//        setPow(0);
    }


    public void collectLowerPixelsFar(Babaji babaji) throws InterruptedException {
        babaji.setMotorPowers(0.0, 0.0, 0.0, 0.0);
        boolean pixelIn = false;
        ElapsedTime timer = new ElapsedTime();
        setPow(collectPow);
        intakePixel3();
//        sleep(300);
//        timer.reset();

        while (!sensor.intakeBackHasPixel()) {
            if (!pixelIn) {
                babaji.setMotorPowers(intakePursuitPow, intakePursuitPow, intakePursuitPow, intakePursuitPow);
                if (sensor.getFrontDistance() <= closeUpDist2){
                    babaji.setMotorPowers(0,0,0,0);
                    intakePixel2();
                    timer.reset();
                    pixelIn = true;
                    while (timer.time(TimeUnit.SECONDS) < stackWaitTime) {}
                    babaji.setMotorPowers(-intakePursuitPow, -intakePursuitPow, -intakePursuitPow, -intakePursuitPow);
                    timer.reset();
                }
            } else {
                if (sensor.getFrontDistance() > backUpDist){
                    intakePixel1();
                    while (timer.time(TimeUnit.SECONDS) < armDropTime){
                        babaji.setMotorPowers(0,0,0,0);
                    }
//                    babaji.setMotorPowers(-intakePursuitPow, -intakePursuitPow, -intakePursuitPow, -intakePursuitPow);
                } else {
                    timer.reset();
                }
            }
        }
        babaji.setMotorPowers(0, 0, 0, 0);
        intakePixel2();
        sleep(500);
        pixelIn = false;
        timer.reset();
        while (!sensor.intakeFrontHasPixel()) {
            if (!pixelIn) {
                babaji.setMotorPowers(intakePursuitPow, intakePursuitPow, intakePursuitPow, intakePursuitPow);
                if (sensor.getFrontDistance() <= closeUpDist2){
                    babaji.setMotorPowers(0,0,0,0);
                    intakePixel1();
                    timer.reset();
                    pixelIn = true;
                    while (timer.time(TimeUnit.SECONDS) < stackWaitTime) {}
                    babaji.setMotorPowers(-intakePursuitPow, -intakePursuitPow, -intakePursuitPow, -intakePursuitPow);
                    timer.reset();
                }
            } else {
                if (sensor.getFrontDistance() > backUpDist){
                    intakePixel1();
                    while (timer.time(TimeUnit.SECONDS) < armDropTime){
                        babaji.setMotorPowers(0,0,0,0);
                    }
//                    babaji.setMotorPowers(-intakePursuitPow, -intakePursuitPow, -intakePursuitPow, -intakePursuitPow);
                } else {
                    timer.reset();
                }
            }
        }
        babaji.setMotorPowers(0,0,0,0);
//        sleep(1000);
//        setPow(0);
    }

//    public void collectPixelsFarCycle(Babaji babaji) throws InterruptedException {
//        babaji.setMotorPowers(0.0, 0.0, 0.0, 0.0);
//        setPow(collectPow);
//        intakePixel4();
//        while (!sensor.intakeBackHasPixel()) {}
//        intakePixel3();
//        while (!sensor.intakeFrontHasPixel()) {}
//        sleep(1000);
//        setPow(0);
//    }
    // THIS FUNCTION WORKS
    public void collectPixel5Fixed(Babaji babaji) throws InterruptedException {
        raiseIntakeMoving();
        sleep(300);
        ElapsedTime timer = new ElapsedTime();
        boolean pixelIn = false;
        babaji.setMotorPowers(0.0, 0.0, 0.0, 0.0);
        setPow(collectPow);
        timer.reset();
        while (!sensor.intakeFrontHasPixel()) {
            if (!pixelIn) {
                babaji.setMotorPowers(intakePursuitPow, intakePursuitPow, intakePursuitPow, intakePursuitPow);
                if (sensor.getFrontDistance() <= closeUpDist){
                    babaji.setMotorPowers(0,0,0,0);
                    intakePixel5();
                    timer.reset();
                    pixelIn = true;
                    while (timer.time(TimeUnit.SECONDS) < stackWaitTime) {}
                    babaji.setMotorPowers(-intakePursuitPow, -intakePursuitPow, -intakePursuitPow, -intakePursuitPow);
                    timer.reset();
                }
            } else {
                if (sensor.getFrontDistance() > backUpDist){
                    intakePixel1();
                    while (timer.time(TimeUnit.SECONDS) < armDropTime){
                        babaji.setMotorPowers(0,0,0,0);
                    }
//                    babaji.setMotorPowers(-intakePursuitPow, -intakePursuitPow, -intakePursuitPow, -intakePursuitPow);
                } else {
                    timer.reset();
                }
            }
        }
        babaji.setMotorPowers(0,0,0,0);
    }

    public void collectPixel1(Babaji babaji) throws InterruptedException {
        ElapsedTime timer = new ElapsedTime();
        babaji.setMotorPowers(0.0, 0.0, 0.0, 0.0);
        setPow(collectPow);
        intakePixel1();
        sleep(300);
        timer.reset();
        while (!sensor.intakeFrontHasPixel() || !sensor.intakeBackHasPixel()) {
            if (timer.time(TimeUnit.SECONDS) > 3) {
                break;
            }
        }
        sleep(1000);
        setPow(0);
    }



    public void incrementRotate(double increment) {
        rightRotate.setPosition(rotateRightUpPos + increment);
        leftRotate.setPosition(rotateLeftUpPos - increment);
    }

    public void collectPixels() throws InterruptedException {
        ElapsedTime timer = new ElapsedTime();
        boolean cont = true;
        setPow(collectPow);
        intakePixel1();
        while (!sensor.intakeBackHasPixel()) {
            if (timer.time(TimeUnit.SECONDS) > 2) {
                setPow(0);
                cont = false;
                break;
            }
        }
        sleep(1000);
        timer.reset();
        if (cont) {
            while (!sensor.intakeFrontHasPixel()) {
                if (timer.time(TimeUnit.SECONDS) > 2) {
                    setPow(0);
                    break;
                }
            }
        }
//        setPow(collectPow * 1.2);
//        sleep(1000);
        setPow(0);
    }

//    public void collectPixels() throws InterruptedException {
//        boolean hasTwoPixels = false;
//
//        setPow(collectPow);
//        while (!hasTwoPixels) {
//            if (sensor.intakeBackHasPixel()) {
//                sleep(500);
//                setPow(collectPow * 0.75);
//                if (sensor.intakeFrontHasPixel()) {
//                    hasTwoPixels = true;
//                }
//            } else {
//                hasTwoPixels = false;
//            }
//        }
//        setPow(collectPow * 0.5);
//        sleep(500);
//        setPow(spitOutPow);
//        sleep(1000);
//        setPow(0);
//    }

    public void setPow(double pow) {
        intake.setPower(pow);
    }

    public void raiseIntake() {
        rightRotate.setPosition(rotateRightUpPos);
        leftRotate.setPosition(rotateLeftUpPos);
    }

    public void raiseIntakeMoving() {
        rightRotate.setPosition(rotateRightIntakeMovingPos);
        leftRotate.setPosition(rotateLeftIntakeMovingPos);
    }

    public void intakePixel5() {
        rightRotate.setPosition(pixel5PosRight);
        leftRotate.setPosition(pixel5PosLeft);
    }

    public void intakePixel4() {
        rightRotate.setPosition(pixel4PosRight);
        leftRotate.setPosition(pixel4PosLeft);
    }

    public void intakePixel3() {
        rightRotate.setPosition(pixel3PosRight);
        leftRotate.setPosition(pixel3PosLeft);
    }

    public void intakePixel2() {
        rightRotate.setPosition(pixel2PosRight);
        leftRotate.setPosition(pixel2PosLeft);
    }

    public void intakePixel1() {
        rightRotate.setPosition(pixel1PosRight);
        leftRotate.setPosition(pixel1PosLeft);
    }

    public void lowerIntake() {
        rightRotate.setPosition(rotateRightDownPos);
        leftRotate.setPosition(rotateLeftDownPos);
    }
}
