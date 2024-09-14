package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.Babaji;
import org.firstinspires.ftc.teamcode.drive.Intake;
import org.firstinspires.ftc.teamcode.drive.Outtake;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.concurrent.TimeUnit;

/**
 * BUTTON MAPPING
 * @Gamepad_1 (Chassis, Drone, Hang)
 *     Joysticks         - Drive
 *     A                 - Slow Drive Toggle
 *     Left Trigger      - Hang Toggle
 *     Left/Right Bumper - Drone Launch
 *
 * @Gamepad_2 (Intake, Transfer, Outtake)
 *     DPad Up          - Slides High
 *     DPad Left        - Slides Mid
 *     DPad Right       - Slides Low
 *     DPad Down        - Slides Down
 *     Right Joystick    - Slides Micro Adjustment
 *     Left Joystick   - Intake Manual
 *     L/R Bumper       - Rotate Jonny Toggle
 *     L/R Trigger      - Front/Back Jonny Score
 *     A                - Auto Intake
 *     Y                - Transfer
 *     B                - Raise Intake So Chassis Can Move
 *
 */

@Config
@TeleOp
public class InkOp extends LinearOpMode {
    private Babaji babaji;
    private Outtake.DepositLevel outtakeLevel;

    public static double driveMultiplierSlow = 0.4, driveMultiplierNormal = 1.0, driveMultiplierSlower = 0.2;
    public static double frontDistanceThresh = 30;

    public static double frontDistanceBlock = 15.0;
    public static double distanceDtThresh = 200;
    public static double timerThresh = 100;
    private double driveMultiplier;

    public double currentBackDistance;

    enum TransferState {
        IDLE,
        STAGE_1,
        STAGE_2,
        STAGE_3
    }
    enum SlideState {
        IDLE,
        LOW,
        MID,
        HIGH,
        GROUND_S1,
        GROUND_S2,
        RESET_ZERO
    }
    enum IntakeState {
        IDLE,
        STAGE_1,
        STAGE_2,
        STAGE_3,
        STAGE_4
    }

    TransferState currentTransferState = TransferState.IDLE;
    SlideState currentSlideState = SlideState.IDLE;
    IntakeState currentIntakeState = IntakeState.IDLE;

    @Override
    public void runOpMode() throws InterruptedException {
        babaji = new Babaji(hardwareMap);
        babaji.init();
        outtakeLevel = Outtake.DepositLevel.GROUND;

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

//        Collect collect = new Collect();
//        Transfer transfer = new Transfer();
//        RaiseSlides raiseSlides = new RaiseSlides();
//        Chassis chassis = new Chassis();

        boolean isHangUp = false;
        boolean isJonnyBackExtrude = false, isJonnyFrontExtrude = false;
        boolean slowDrive = false;

        boolean gamepad1Button = false;
        boolean gamepad2Button = false;

        double driveDirection;
        double prevBackDistance = 0;

        driveMultiplier = driveMultiplierNormal;
        double driveMultiplierX = 1;

        Outtake.JonnyRotatePos[] jonnyRotatePoses = new Outtake.JonnyRotatePos[]{
                Outtake.JonnyRotatePos.RIGHT,
                Outtake.JonnyRotatePos.VERTICAL,
                Outtake.JonnyRotatePos.LEFT,
                Outtake.JonnyRotatePos.HORIZONTAL,
        };
        int jonnyRotatePosIndex = 1;
        ElapsedTime transferTimer = new ElapsedTime();
        ElapsedTime slidesTimer = new ElapsedTime();
        ElapsedTime retractTimer = new ElapsedTime();
        ElapsedTime intakeTimer = new ElapsedTime();

        Gamepad.LedEffect rgbEffect = new Gamepad.LedEffect.Builder()
                .addStep(1, 0, 0, 250) // Show red for 250ms
                .addStep(0, 1, 0, 250) // Show green for 250ms
                .addStep(0, 0, 1, 250) // Show blue for 250ms
                .addStep(1, 1, 1, 250) // Show white for 250ms
                .build();

        waitForStart();

        gamepad1.runLedEffect(rgbEffect);
        gamepad1.rumble(1000);

        ElapsedTime backdropLoop = new ElapsedTime();
        double distanceTotal = 0;
        int distanceCounter = 0;
        boolean improperSet = true;

//        chassis.start();

        while (!isStopRequested()) {
            /** @Gamepad_1 (Chassis, Drone, Hang) */
//            if (gamepad1.a) {
//                // Slow Drive Toggle
//                slowDrive = !slowDrive;
//                driveMultiplier = slowDrive ? driveMultiplierSlow : driveMultiplierNormal;
////                while (gamepad1.a) {}
//            }

//            if (backdropLoop.time(TimeUnit.MILLISECONDS) < timerThresh) {
//                distanceTotal += currentBackDistance;
//                distanceCounter++;
//                if (distanceTotal / distanceCounter > distanceDtThresh) {
//                    distanceTotal = 0;
//                    distanceCounter = 0;
//                    backdropLoop.reset();
//                }
//            } else {
//                if (!(distanceTotal / distanceCounter > distanceDtThresh)) {
//                    if (currentBackDistance < frontDistanceBlock) {
//                        driveMultiplierX = 0;
//                    } else if (currentBackDistance < frontDistanceThresh) {
//                        driveMultiplier = driveMultiplierSlow;
//                        driveMultiplierX = driveMultiplierNormal;
//                    } else {
//                        driveMultiplier = driveMultiplierNormal;
//                        driveMultiplierX = driveMultiplierNormal;
//                    }
//                } else {
//                    driveMultiplier = driveMultiplierNormal;
//                    driveMultiplierX = driveMultiplierNormal;
//                }
//                distanceTotal = 0;
//                distanceCounter = 0;
//                backdropLoop.reset();
//            }
//                if (gamepad1.right_trigger > 0.2 || Math.abs(currentBackDistance - prevBackDistance) > distanceDtThresh) {
//                    driveMultiplier = driveMultiplierNormal;
//                    driveMultiplierX = driveMultiplierNormal;
//                }

//            if (currentBackDistance < 200) {
//                if (currentBackDistance < frontDistanceBlock) {
//                    driveMultiplierX = 0;
//                } else if (currentBackDistance < frontDistanceThresh) {
//                    driveMultiplier = driveMultiplierSlow;
//                    driveMultiplierX = driveMultiplierNormal;
//                } else {
//                    driveMultiplier = driveMultiplierNormal;
//                    driveMultiplierX = driveMultiplierNormal;
//                }
//            } else {
//                driveMultiplier = driveMultiplierNormal;
//                driveMultiplierX = driveMultiplierNormal;
//            }


            if (gamepad1.right_trigger > 0.4) {
                double currentBackDistance = babaji.sensors.getBackDistance();
                if (currentBackDistance < frontDistanceBlock) {
                    driveMultiplier = driveMultiplierSlower;
                    driveMultiplierX = 0;
                } else if (currentBackDistance < frontDistanceThresh) {
                    driveMultiplier = driveMultiplierSlower;
                    driveMultiplierX = 0;
                } else {
                    driveMultiplier = driveMultiplierSlow;
                    driveMultiplierX = 1;
                }
            } else {
                driveMultiplier = driveMultiplierNormal;
                driveMultiplierX = 1;
            }

            babaji.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y * driveMultiplier * driveMultiplierX,
                            -gamepad1.left_stick_x * driveMultiplier,
                            -gamepad1.right_stick_x * driveMultiplier
                    )
            );
            babaji.update();

            if (gamepad1.left_trigger > 0.2) {
                babaji.intake.raiseIntake();
                // Hang Toggle
                if (isHangUp) {
                    babaji.hang.lower();
                } else {
                    babaji.hang.raise();
                }

                isHangUp = !isHangUp;
//                while (gamepad1.left_trigger > 0.2) {}
            }
            if (gamepad1.left_bumper && gamepad1.right_bumper) {
                babaji.droneLauncher.release();
//                while (gamepad1.left_bumper && gamepad1.right_bumper) {}
            }

//            if (gamepad2.left_trigger > 0.2) {
////                collect.start();
//                babaji.collectGround();
//            }
//            if (gamepad2.dpad_down) {
////                transfer.start();
//                babaji.transfer();
//            }
//            if (gamepad2.dpad_up) {
//                babaji.outtake.setJonnyExtrudePos(Outtake.JonnyExtrudePos.COLLECT);
//            }

            /** @Gamepad_2 (Intake, Transfer, Outtake) */
//            if (gamepad2.dpad_up || gamepad2.dpad_left || gamepad2.dpad_right && !gamepad2Button) {
//                gamepad2Button = true;
//                babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.BACKDROP);
//                babaji.outtake.setJonnyRotate(Outtake.JonnyRotatePos.VERTICAL);
//                jonnyRotatePosIndex = 1;
//                babaji.outtake.setJonnyExtrudePos(Outtake.JonnyExtrudePos.COLLECT);
//            }
//            if (gamepad2.dpad_down && !gamepad2Button) {
//                gamepad2Button = true;
//                babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.SHIELD);
//                babaji.outtake.setJonnyRotate(Outtake.JonnyRotatePos.VERTICAL);
//                babaji.outtake.setJonnyExtrudePos(Outtake.JonnyExtrudePos.SCORE);
//                jonnyRotatePosIndex = 1;
//            }

            if (gamepad2.dpad_up && !gamepad2Button) {
                gamepad2Button = true;
                babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.BACKDROP);
                babaji.outtake.setJonnyRotate(Outtake.JonnyRotatePos.VERTICAL);
                jonnyRotatePosIndex = 1;
                babaji.outtake.setJonnyExtrudePos(Outtake.JonnyExtrudePos.COLLECT);
                // Slides High
                currentSlideState = SlideState.HIGH;
                slidesTimer.reset();
            }
            if (gamepad2.dpad_left && !gamepad2Button) {
                gamepad2Button = true;
                babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.BACKDROP);
                babaji.outtake.setJonnyRotate(Outtake.JonnyRotatePos.VERTICAL);
                jonnyRotatePosIndex = 1;
                babaji.outtake.setJonnyExtrudePos(Outtake.JonnyExtrudePos.COLLECT);
                // Slides Mid
                currentSlideState = SlideState.MID;
                slidesTimer.reset();
            } else if (gamepad2.dpad_right && !gamepad2Button) {
                gamepad2Button = true;
                babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.BACKDROP);
                babaji.outtake.setJonnyRotate(Outtake.JonnyRotatePos.VERTICAL);
                jonnyRotatePosIndex = 1;
                babaji.outtake.setJonnyExtrudePos(Outtake.JonnyExtrudePos.COLLECT);
                // Slides Low
                currentSlideState = SlideState.LOW;
                slidesTimer.reset();
            } else if (gamepad2.dpad_down && !gamepad2Button) {
                gamepad2Button = true;
                // Slides Down
                currentSlideState = SlideState.GROUND_S1;
                slidesTimer.reset();
            }
            if (Math.abs(gamepad2.right_stick_y) > 0.3 && currentSlideState == SlideState.IDLE) {
                // Slides Micro Adjustment
                babaji.outtake.setSlidesPow(gamepad2.right_stick_y * -1);
            } else if (currentSlideState == SlideState.IDLE) {
                babaji.outtake.setSlidesPow(Outtake.restPow);
            }
            if (gamepad2.left_bumper && !gamepad2Button) {
                gamepad2Button = true;
                // Rotate Jonny Clockwise Toggle
                jonnyRotatePosIndex--;
                if (jonnyRotatePosIndex < 0) {
                    jonnyRotatePosIndex = 0;
                }
                babaji.outtake.setJonnyRotate(jonnyRotatePoses[jonnyRotatePosIndex]);
//                while (gamepad2.left_bumper) {}
            }
            if (gamepad2.right_bumper && !gamepad2Button) {
                gamepad2Button = true;
                // Rotate Jonny Counter-Clockwise Toggle
                jonnyRotatePosIndex++;
                if (jonnyRotatePosIndex >= jonnyRotatePoses.length) {
                    jonnyRotatePosIndex = jonnyRotatePoses.length-1;
                }
                babaji.outtake.setJonnyRotate(jonnyRotatePoses[jonnyRotatePosIndex]);
//                while (gamepad2.right_bumper) {}
            }
            if (gamepad2.left_trigger > 0.2 && gamepad2.right_trigger > 0.2 && !gamepad2Button) {
                gamepad2Button = true;
                // Front Jonny Score
                isJonnyFrontExtrude = !isJonnyFrontExtrude;
                isJonnyBackExtrude = !isJonnyBackExtrude;
                if (isJonnyFrontExtrude) {
                    babaji.outtake.setJonnyFrontExtrudePos(Outtake.JonnyExtrudePos.SCORE);
                } else {
                    babaji.outtake.setJonnyFrontExtrudePos(Outtake.JonnyExtrudePos.COLLECT);
                }
                if (isJonnyBackExtrude) {
                    babaji.outtake.setJonnyBackExtrudePos(Outtake.JonnyExtrudePos.SCORE);
                } else {
                    babaji.outtake.setJonnyBackExtrudePos(Outtake.JonnyExtrudePos.COLLECT);
                }
//                while (gamepad2.left_trigger > 0.2 || gamepad2.right_trigger > 0.2) {}
            } else if (gamepad2.left_trigger > 0.2 && !gamepad2Button) {
                gamepad2Button = true;
                // Front Jonny Score
                isJonnyFrontExtrude = !isJonnyFrontExtrude;
                if (isJonnyFrontExtrude) {
                    babaji.outtake.setJonnyFrontExtrudePos(Outtake.JonnyExtrudePos.SCORE);
                } else {
                    babaji.outtake.setJonnyFrontExtrudePos(Outtake.JonnyExtrudePos.COLLECT);
                }
//                while (gamepad2.left_trigger > 0.2) {}
            } else if (gamepad2.right_trigger > 0.2 && !gamepad2Button) {
                // Back Jonny Score
                gamepad2Button = true;
                isJonnyBackExtrude = !isJonnyBackExtrude;
                if (isJonnyBackExtrude) {
                    babaji.outtake.setJonnyBackExtrudePos(Outtake.JonnyExtrudePos.SCORE);
                } else {
                    babaji.outtake.setJonnyBackExtrudePos(Outtake.JonnyExtrudePos.COLLECT);
                }
//                while (gamepad2.right_trigger > 0.2) {}
            }
            if (gamepad2.a && !gamepad2Button) {
                // Auto Ground Intake
                currentIntakeState = IntakeState.STAGE_1;
                gamepad2Button = true;
            }
            if (gamepad2.left_stick_y > 0.5 && !gamepad2Button) {
                currentIntakeState = IntakeState.IDLE;
                gamepad2Button = true;
            }

            if (currentIntakeState == IntakeState.IDLE) {
                babaji.intake.setPow(gamepad2.left_stick_y);
            }

            if (gamepad2.y && !gamepad2Button) {
                gamepad2Button = true;
                currentTransferState = TransferState.STAGE_1;
            }
            if (gamepad2.b && !gamepad2Button) {
                gamepad2Button = true;
                babaji.intake.raiseIntakeMoving();
//                while (gamepad2.b) {}
//                sleep(500);
            }

            switch (currentTransferState) {
                case STAGE_1:
                    babaji.intake.setPow(Intake.alignPow);
                    babaji.intake.raiseIntakeMoving();
//                    babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.SHIELD);
                    babaji.outtake.setJonnyExtrudePos(Outtake.JonnyExtrudePos.COLLECT);
                    babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.PREP);
                    currentTransferState = TransferState.STAGE_2;
                    transferTimer.reset();
                    break;
                case STAGE_2:
                    if (transferTimer.time(TimeUnit.MILLISECONDS) > 350) {
                        babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.INTAKE);
                        currentTransferState = TransferState.STAGE_3;
                        transferTimer.reset();
                    }
                    break;
                case STAGE_3:
                    babaji.intake.setPow(0);
                    if (transferTimer.time(TimeUnit.MILLISECONDS) > 500) {
                        babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.SHIELD);
                        currentTransferState = TransferState.IDLE;
                    }
                    break;
                case IDLE:
                    break;
            }

            // At any time these slide movements need to be interrupted, we need to create a state that allows that or something like that
            switch (currentSlideState) {
                case LOW:
//                    driveDirection = 1;//(Outtake.low - babaji.outtake.getEncCount())/Math.abs(Outtake.low - babaji.outtake.getEncCount());
                    driveDirection = babaji.outtake.getEncCount() < Outtake.low ? 1 : -1;
                    if (Math.abs(babaji.outtake.getEncCount() - Outtake.low) > Outtake.allowableError) {
                        if (driveDirection == 1) {
                            babaji.outtake.setSlidesPow(Outtake.slidesPosPow);
                        } else {
                            babaji.outtake.setSlidesPow(Outtake.slidesNegPow);
                        }
                        if (slidesTimer.time(TimeUnit.SECONDS) > 1.5) {
                            babaji.outtake.setSlidesPow(Outtake.restPow);
                            currentSlideState = SlideState.IDLE;
                        }
                    } else {
                        babaji.outtake.setSlidesPow(Outtake.restPow);
                        currentSlideState = SlideState.IDLE;
                    }
                    break;
                case MID:
//                    driveDirection = 1;//(Outtake.mid - babaji.outtake.getEncCount())/Math.abs(Outtake.mid - babaji.outtake.getEncCount());
                    driveDirection = babaji.outtake.getEncCount() < Outtake.mid ? 1 : -1;
                    if (Math.abs(babaji.outtake.getEncCount() - Outtake.mid) > Outtake.allowableError) {
                        if (driveDirection == 1) {
                            babaji.outtake.setSlidesPow(Outtake.slidesPosPow);
                        } else {
                            babaji.outtake.setSlidesPow(Outtake.slidesNegPow);
                        }
                        if (slidesTimer.time(TimeUnit.SECONDS) > 1.5) {
                            babaji.outtake.setSlidesPow(Outtake.restPow);
                            currentSlideState = SlideState.IDLE;
                        }
                    } else {
                        babaji.outtake.setSlidesPow(Outtake.restPow);
                        currentSlideState = SlideState.IDLE;
                    }
                    break;
                case HIGH:
//                    driveDirection = 1;//(Outtake.high - babaji.outtake.getEncCount())/Math.abs(Outtake.high - babaji.outtake.getEncCount());
                    driveDirection = babaji.outtake.getEncCount() < Outtake.high ? 1 : -1;
                    if (Math.abs(babaji.outtake.getEncCount() - Outtake.high) > Outtake.allowableError) {
                        if (driveDirection == 1) {
                            babaji.outtake.setSlidesPow(Outtake.slidesPosPow);
                        } else {
                            babaji.outtake.setSlidesPow(Outtake.slidesNegPow);
                        }
                        if (slidesTimer.time(TimeUnit.SECONDS) > 1.5) {
                            babaji.outtake.setSlidesPow(Outtake.restPow);
                            currentSlideState = SlideState.IDLE;
                        }
                    } else {
                        babaji.outtake.setSlidesPow(Outtake.restPow);
                        currentSlideState = SlideState.IDLE;
                    }
                    break;
                case GROUND_S1:
                    babaji.outtake.setJonnyRotate(Outtake.JonnyRotatePos.VERTICAL);
                    babaji.outtake.setJonnyExtrudePos(Outtake.JonnyExtrudePos.SCORE);
                    jonnyRotatePosIndex = 1;
                    retractTimer.reset();
                    currentSlideState = SlideState.GROUND_S2;
                case GROUND_S2:
                    if (retractTimer.time(TimeUnit.MILLISECONDS) > 200) {
                        babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.SHIELD);
                        driveDirection = (Outtake.ground - babaji.outtake.getEncCount()) / Math.abs(Outtake.ground - babaji.outtake.getEncCount());
                        if (Math.abs(babaji.outtake.getEncCount() - Outtake.ground) > Outtake.allowableError) {
                            if (driveDirection == 1) {
                                babaji.outtake.setSlidesPow(Outtake.slidesPosPow);
                            } else {
                                babaji.outtake.setSlidesPow(Outtake.slidesNegPow);
                            }
                            if (slidesTimer.time(TimeUnit.SECONDS) > 1.5) {
                                babaji.outtake.setSlidesPow(Outtake.restPow);
                                currentSlideState = SlideState.IDLE;
                            }
                        } else {
                            babaji.outtake.setSlidesPow(Outtake.restPow);
                            slidesTimer.reset();
                            currentSlideState = SlideState.RESET_ZERO;
                        }
                    }
                    break;
                case RESET_ZERO:
                    if (slidesTimer.time(TimeUnit.SECONDS) > 0.5) {
                        babaji.outtake.setSlidesPow(0);
                        currentSlideState = SlideState.IDLE;
                    }
                    break;
                case IDLE:
                    break;
            }

            switch (currentIntakeState) {
                case STAGE_1:
                    babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.SHIELD);
                    babaji.outtake.setJonnyExtrudePos(Outtake.JonnyExtrudePos.SCORE);
                    babaji.intake.intakePixel1();
                    intakeTimer.reset();
                    currentIntakeState = IntakeState.STAGE_2;
                    break;
                case STAGE_2:
                    babaji.intake.setPow(Intake.collectPow);
                    if (babaji.sensors.intakeBackHasPixel()) {
                        intakeTimer.reset();
                        currentIntakeState = IntakeState.STAGE_3;
                    }
                    break;
                case STAGE_3:
                    if ((intakeTimer.time(TimeUnit.MILLISECONDS) > 500) && babaji.sensors.intakeFrontHasPixel()) {
                        // rumble d1 controller
                        babaji.intake.raiseIntakeMoving();
                        intakeTimer.reset();
                        currentIntakeState = IntakeState.STAGE_4;
                    }
                    break;
                case STAGE_4:
                    if (intakeTimer.time(TimeUnit.SECONDS) > 1) {
                        currentTransferState = TransferState.STAGE_1;
                        currentIntakeState = IntakeState.IDLE;
                        intakeTimer.reset();
                    }
                    break;
                case IDLE:
                    break;
            }

            telemetry.addData("Drive Mutiplier", driveMultiplier);
            telemetry.addData("currentBackDistance", currentBackDistance);
            telemetry.addData("prevBackDistance", prevBackDistance);
            telemetry.addData("Slow Drive", slowDrive);
            telemetry.addData("Slides Enc Count", babaji.outtake.getEncCount());
            telemetry.addData("Intake Stage", currentIntakeState);
            telemetry.update();

            if (!gamepad2.a && !gamepad2.b && !gamepad2.x && !gamepad2.y && !gamepad2.left_bumper && !gamepad2.right_bumper && gamepad2.right_trigger < 0.2 && gamepad2.left_trigger < 0.2 && !gamepad2.dpad_up && !gamepad2.dpad_down && !gamepad2.dpad_right && !gamepad2.dpad_left) {
                gamepad2Button = false;
            }

        }
        babaji.setMotorPowers(0,0,0,0);
        babaji.stop();
    }

//    private class Collect extends Thread {
//        @Override
//        public void run() {
//            try {
//                babaji.collectGroundMoving();
//            }
//            catch (InterruptedException e) {}
//            this.interrupt();
////            babaji.intake.setPow(0.0);
//        }
//    }

//    private class Transfer extends Thread {
//        @Override
//        public void run() {
//            try {
//                babaji.transfer();
//            } catch (InterruptedException e) {}
//            this.interrupt();
//        }
//    }

//    private class RaiseSlides extends Thread {
//        @Override
//        public void run() {
//            try {
//                babaji.outtake.raiseToHeight(outtakeLevel);
//            } catch (InterruptedException e) {
//                throw new RuntimeException(e);
//            }
//
//            this.interrupt();
//            babaji.outtake.setSlidesPow(Outtake.restPow);
//        }
//    }

//    private class Chassis extends Thread {
//        @Override
//        public void run() {
//            try {
//                while (opModeIsActive()) {
//                    babaji.setWeightedDrivePower(
//                            new Pose2d(
//                                    -gamepad1.left_stick_y * driveMultiplier,
//                                    -gamepad1.left_stick_x * driveMultiplier,
//                                    -gamepad1.right_stick_x * driveMultiplier
//                            )
//                    );
//                    babaji.update();
//                }
//            } catch (Exception e) {
//            }
//
//            this.interrupt();
//        }
//    }
}