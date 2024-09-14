package org.firstinspires.ftc.teamcode.drive;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.Subsystem;

import java.util.concurrent.TimeUnit;

@Config
public class Outtake extends Subsystem {

    /** Hardware/Constants */
    private DcMotorEx slidesLeft, slidesRight;
    private Encoder slidesEncoder;
    private Servo rotateRight, rotateLeft, actuator, jonnyRotate, jonnyBack, jonnyFront;
    private Sensors sensor;

    public static double kP = 0.0, kI = 0.0, kD = 0.0;
    public static double restPow = 0.05, maxPow = 0.9;
    public static double allowableError = 2000;
    private PIDController pidController;

    public static int ground = 0;
    public static int low = 15000;
    public static int mid = 25000;
    public static int high = 40000;
    public static double slidesPosPow = 1;
    public static double slidesNegPow = -0.6;

//    public static double slidesEncoderSlip = 0.0;

    public static double jonnyBackCollectPos = 0, jonnyFrontCollectPos = 0, jonnyBackScorePos = 1, jonnyFrontScorePos = 1;
    public static double jonnyIntakePos = 0.69;
    public static double jonnyVerticalPos = 0.69;
    public static double jonnyRightPos = 0.85;
    public static double jonnyLeftPos = 0.5;
    public static double jonnyGroundPos = 0.0;
    public static double jonnyHorizontalPos = 0.36;

    public static double rotateLeftGroundPos = 0.14;
    public static double rotateRightGroundPos = 0.17;
    public static double rotateLeftIntakePos = 0.81;
    public static double rotateRightIntakePos = 0.84;
    public static double rotateRightPrepPos = 0.59;
    public static double rotateLeftPrepPos = 0.62;
    public static double rotateLeftBackdropPos = 0.28;
    public static double rotateRightBackdropPos = 0.31;
    public static double rotateLeftScoreStepPos = 0.39;
    public static double rotateRightScoreStepPos = 0.42;
    public static double rotateLeftShieldPos = 0.7;
    public static double rotateRightShieldPos = 0.7;
    public static double rotateLeftRestPos = 0.63;
    public static double rotateRightRestPos = 0.58;

    public static double actuatorShieldPos = 0.22; //0.56;
    public static double actuatorIntakePos = 0.22;
    public static double actuatorGroundPos = 0.22;
    public static double actuatorBackdropPos = 0;
    public static double actuatorTransferPos = 0.75;

    public enum DepositLevel {
        GROUND,
        LOW,
        MID,
        HIGH
    }

    public enum JonnyRotatePos {
        INTAKE,
        VERTICAL,
        RIGHT,
        LEFT,
        GROUND,
        HORIZONTAL
    }
    public enum OuttakeRotatePos {
        PREP,
        INTAKE,
        GROUND,
        BACKDROP,
        SHIELD,
        REST,
        SCORE_STEP
    }

    public enum ActuatorRotatePos {
        INTAKE,
        GROUND,
        BACKDROP,
        SHIELD,
        TRANSFER
    }
    public enum JonnyExtrudePos {
        COLLECT,
        SCORE
    }

    public Outtake(HardwareMap hardwareMap, Sensors sensor) {
        slidesLeft = hardwareMap.get(DcMotorEx.class, "slidesLeft");
        slidesEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "slidesLeft"));
        slidesEncoder.setDirection(Encoder.Direction.REVERSE);

        slidesRight = hardwareMap.get(DcMotorEx.class, "slidesRight");
        slidesRight.setDirection(DcMotorSimple.Direction.REVERSE); // up is positive, down is negative

        slidesRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slidesRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slidesLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slidesLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        slidesEncoder.setDirection(Encoder.Direction.REVERSE);
//        slidesEncoderSlip = slidesLeft.getCurrentPosition();

        rotateRight = hardwareMap.get(Servo.class, "outtakeRotateRight");
        rotateLeft = hardwareMap.get(Servo.class, "outtakeRotateLeft");
        actuator = hardwareMap.get(Servo.class, "outtakeActuator");
        jonnyRotate = hardwareMap.get(Servo.class, "outtakeJonnyRotate");
        jonnyBack = hardwareMap.get(Servo.class, "outtakeJonnyFront"); // don't change this, back is front
        jonnyFront = hardwareMap.get(Servo.class, "outtakeJonnyBack");

        this.pidController = new PIDController(kP, kI, kD, maxPow);
        this.sensor = sensor;
    }

    /** Base Functions */
    @Override
    public void init() {
        setOuttakeRotate(OuttakeRotatePos.SHIELD);
        setJonnyRotate(JonnyRotatePos.VERTICAL);
        setJonnyExtrudePos(JonnyExtrudePos.SCORE);
    }

    @Override
    public void stop() {

    }

    /** Mechanism Functions */
    public void setSlidesPow(double pow) {
        slidesLeft.setPower(pow);
        slidesRight.setPower(pow);
    }

    public double getEncCount() {
        // Negative as it goes up, max near -4000
//        return slidesLeft.getCurrentPosition();
        return slidesEncoder.getCurrentPosition();
    }

    public void raiseToHeight(DepositLevel level) throws InterruptedException {
        if (level == DepositLevel.GROUND) {
            raiseToHeight(ground);
//            slidesEncoderSlip = slidesEncoder.getCurrentPosition();
        }
        else if (level == DepositLevel.LOW) {
            raiseToHeight(low);
        }
        else if (level == DepositLevel.MID) {
            raiseToHeight(mid);
        }
        else if (level == DepositLevel.HIGH) {
            raiseToHeight(high);
        }

//        levels.put(DepositLevel.GROUND, ground);
//        levels.put(DepositLevel.LOW, low);
//        levels.put(DepositLevel.MID, mid);
//        levels.put(DepositLevel.HIGH, high);
    }

    public void raiseToHeight(double encCount) throws InterruptedException {
        ElapsedTime timer = new ElapsedTime();
        double driveDirection = (encCount - this.getEncCount())/Math.abs(encCount - this.getEncCount());

        while (Math.abs(this.getEncCount() - encCount) > allowableError) {
            if (driveDirection == 1) {
                setSlidesPow(slidesPosPow);
            } else {
                setSlidesPow(slidesNegPow);
            }
            if (timer.time(TimeUnit.SECONDS) > 1.5) {
                setSlidesPow(restPow);
                break;
            }
//            if (encCount == ground && slidesEncoder.getCorrectedVelocity() < 0.1) {
//                outtakeEncoderSlip = getEncCount();
//            }
        }
        setSlidesPow(restPow);
        if (encCount == ground) {
            setSlidesPow(0.5*slidesNegPow);
            sleep(500);
            setSlidesPow(0);
        }
    }


//    public boolean raiseToHeight(DepositLevel level) throws InterruptedException {
//        double encCount = 0;
//        if (level == DepositLevel.GROUND) {
//            encCount = ground;
////            slidesEncoderSlip = slidesEncoder.getCurrentPosition();
//        }
//        else if (level == DepositLevel.LOW) {
//            encCount = low;
//        }
//        else if (level == DepositLevel.MID) {
//            encCount = mid;
//        }
//        else if (level == DepositLevel.HIGH) {
//            encCount = high;
//        }
//        ElapsedTime timer = new ElapsedTime();
//        double driveDirection = (encCount - this.getEncCount())/Math.abs(encCount - this.getEncCount());
//
//        if (Math.abs(this.getEncCount() - encCount) > allowableError) {
//            if (driveDirection == 1) {
//                setSlidesPow(slidesPosPow);
//            } else {
//                setSlidesPow(slidesNegPow);
//            }
//            if (timer.time(TimeUnit.SECONDS) > 1.5) {
//                setSlidesPow(restPow);
//            }
////            if (encCount == ground && slidesEncoder.getCorrectedVelocity() < 0.1) {
////                outtakeEncoderSlip = getEncCount();
////            }
//        } else {
//            setSlidesPow(restPow);
//            if (encCount == ground) {
//                setSlidesPow(0.5*slidesNegPow);
//                sleep(500);
//                setSlidesPow(0);
//            }
//            return false;
//        }
//        return true;
//    }

    public double getSlidesCurrent() {
        return this.slidesLeft.getCurrent(CurrentUnit.MILLIAMPS);
    }

//    public void raiseToHeight(DepositLevel level) throws InterruptedException {
//        if (level == DepositLevel.GROUND) {
//            raiseToHeight(ground);
////            slidesEncoderSlip = slidesEncoder.getCurrentPosition();
//        }
//        else if (level == DepositLevel.LOW) {
//            raiseToHeight(low);
//        }
//        else if (level == DepositLevel.MID) {
//            raiseToHeight(mid);
//        }
//        else if (level == DepositLevel.HIGH) {
//            raiseToHeight(high);
//        }

//        levels.put(DepositLevel.GROUND, ground);
//        levels.put(DepositLevel.LOW, low);
//        levels.put(DepositLevel.MID, mid);
//        levels.put(DepositLevel.HIGH, high);
//    }

    public void extrude() {
        jonnyBack.setPosition(jonnyBackCollectPos);
        jonnyFront.setPosition(jonnyFrontCollectPos);
    }

    public void retract() {
        jonnyBack.setPosition(jonnyBackScorePos);
        jonnyFront.setPosition(jonnyFrontScorePos);
    }

    public void setJonnyRotate(JonnyRotatePos pos) {
        switch (pos) {
            case GROUND:
                jonnyRotate.setPosition(jonnyGroundPos);
                break;
            case HORIZONTAL:
                jonnyRotate.setPosition(jonnyHorizontalPos);
                break;
            case VERTICAL:
                jonnyRotate.setPosition(jonnyVerticalPos);
                break;
            case INTAKE:
                jonnyRotate.setPosition(jonnyIntakePos);
                break;
            case LEFT:
                jonnyRotate.setPosition(jonnyLeftPos);
                break;
            case RIGHT:
                jonnyRotate.setPosition(jonnyRightPos);
                break;
        }
    }

    public void setOuttakeRotate(OuttakeRotatePos pos) {
        switch (pos) {
            case GROUND:
                rotateLeft.setPosition(rotateLeftGroundPos);
                rotateRight.setPosition(rotateRightGroundPos);
                actuator.setPosition(actuatorGroundPos);
                break;
            case PREP:
                rotateLeft.setPosition(rotateLeftPrepPos);
                rotateRight.setPosition(rotateRightPrepPos);
                actuator.setPosition(actuatorIntakePos);
                break;
            case INTAKE:
                rotateLeft.setPosition(rotateLeftIntakePos);
                rotateRight.setPosition(rotateRightIntakePos);
                actuator.setPosition(actuatorIntakePos);
                break;
            case BACKDROP:
                rotateLeft.setPosition(rotateLeftBackdropPos);
                rotateRight.setPosition(rotateRightBackdropPos);
                actuator.setPosition(actuatorBackdropPos); // actuatorTransferPos
                break;
            case SHIELD:
                rotateLeft.setPosition(rotateLeftShieldPos);
                rotateRight.setPosition(rotateRightShieldPos);
                actuator.setPosition(actuatorShieldPos);
                break;
            case REST:
                rotateLeft.setPosition(rotateLeftRestPos);
                rotateRight.setPosition(rotateRightRestPos);
                actuator.setPosition(actuatorShieldPos);
                break;
            case SCORE_STEP:
                rotateLeft.setPosition(rotateLeftScoreStepPos);
                rotateRight.setPosition(rotateRightScoreStepPos);
                actuator.setPosition(actuatorBackdropPos);
        }
    }

    public void setActuatorRotate(ActuatorRotatePos pos) {
        switch (pos) {
            case GROUND:
                actuator.setPosition(actuatorGroundPos);
                break;
            case INTAKE:
                actuator.setPosition(actuatorIntakePos);
                break;
            case SHIELD:
                actuator.setPosition(actuatorShieldPos);
                break;
            case BACKDROP:
                actuator.setPosition(actuatorBackdropPos);
                break;
            case TRANSFER:
                actuator.setPosition(actuatorTransferPos);
                break;
        }
    }

    public void setJonnyExtrudePos(JonnyExtrudePos pos) {
        switch (pos) {
            case COLLECT:
                jonnyBack.setPosition(jonnyBackCollectPos);
                jonnyFront.setPosition(jonnyFrontCollectPos);
                break;
            case SCORE:
                jonnyBack.setPosition(jonnyBackScorePos);
                jonnyFront.setPosition(jonnyFrontScorePos);
                break;
        }
    }

    public void setJonnyFrontExtrudePos(JonnyExtrudePos pos) {
        switch (pos) {
            case COLLECT:
                jonnyFront.setPosition(jonnyFrontCollectPos);
                break;
            case SCORE:
                jonnyFront.setPosition(jonnyFrontScorePos);
                break;
        }
    }

    public void setJonnyBackExtrudePos(JonnyExtrudePos pos) {
        switch (pos) {
            case COLLECT:
                jonnyBack.setPosition(jonnyBackCollectPos);
                break;
            case SCORE:
                jonnyBack.setPosition(jonnyBackScorePos);
                break;
        }
    }

    public void score() throws InterruptedException {
        setJonnyExtrudePos(JonnyExtrudePos.COLLECT);
        sleep(1000);
        setJonnyExtrudePos(JonnyExtrudePos.SCORE);
    }
}
