package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.Outtake;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;

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
public class AsynkOp extends LinearOpMode {
    private SampleMecanumDriveCancelable babaji;
    private Outtake.DepositLevel outtakeLevel;

    public static double driveMultiplierSlow = 0.4, driveMultiplierNormal = 0.8;
    private double driveMultiplier;

    public enum Mode {
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL
    }
    public static double backdropX = -50, backdropY = 75, backdropAngle = 45;
    public static double intakeX = -115, intakeY = -10, intakeAngle = 180;

    @Override
    public void runOpMode() throws InterruptedException {
        babaji = new SampleMecanumDriveCancelable(hardwareMap);
        babaji.init();
        outtakeLevel = Outtake.DepositLevel.GROUND;

        Collect collect = new Collect();
        Transfer transfer = new Transfer();
        RaiseSlides raiseSlides = new RaiseSlides();
        Chassis chassis = new Chassis();

        boolean isHangUp = false;
        boolean isJonnyBackExtrude = false, isJonnyFrontExtrude = false;
        boolean slowDrive = false;

        Mode currentMode = Mode.DRIVER_CONTROL;

        driveMultiplier = driveMultiplierNormal;

        Outtake.JonnyRotatePos[] jonnyRotatePoses = new Outtake.JonnyRotatePos[]{
                Outtake.JonnyRotatePos.RIGHT,
                Outtake.JonnyRotatePos.VERTICAL,
                Outtake.JonnyRotatePos.LEFT,
                Outtake.JonnyRotatePos.HORIZONTAL,
        };
        int jonnyRotatePosIndex = 1;

        waitForStart();

//        chassis.start();

        while (!isStopRequested()) {

            Pose2d poseEstimate = babaji.getPoseEstimate();

            switch (currentMode) {
                case DRIVER_CONTROL:
                    babaji.setWeightedDrivePower(
                            new Pose2d(
                                    -gamepad1.left_stick_y,
                                    -gamepad1.left_stick_x,
                                    -gamepad1.right_stick_x
                            )
                    );

                    if (gamepad1.a) {
                        // If the A button is pressed on gamepad1, we generate a splineTo()
                        // trajectory on the fly and follow it
                        // We switch the state to AUTOMATIC_CONTROL

                        Trajectory traj1 = babaji.trajectoryBuilder(poseEstimate)
                                .splineTo(new Vector2d(backdropX, backdropY), Math.toRadians(backdropAngle))
                                .build();

                        babaji.followTrajectoryAsync(traj1);

                        currentMode = Mode.AUTOMATIC_CONTROL;
                        while (gamepad1.a) {
                        }
                    } else if (gamepad1.y) {
                        Trajectory traj2 = babaji.trajectoryBuilder(poseEstimate)
                                .splineTo(new Vector2d(intakeX, intakeY), Math.toRadians(intakeAngle))
                                .build();

                        babaji.followTrajectoryAsync(traj2);
                        currentMode = Mode.AUTOMATIC_CONTROL;
                        while (gamepad1.y) {
                        }
                    }
                    break;
                case AUTOMATIC_CONTROL:
                    // If x is pressed, we break out of the automatic following
                    if (Math.abs(gamepad1.left_stick_y) > 0.5) {
                        babaji.breakFollowing();
                        currentMode = Mode.DRIVER_CONTROL;
                    }
                    // If drive finishes its task, cede control to the driver
                    if (!babaji.isBusy()) {
                        currentMode = Mode.DRIVER_CONTROL;
                    }
            }


            /** @Gamepad_1 (Chassis, Drone, Hang) */
            if (gamepad1.a) {
                // Slow Drive Toggle
                slowDrive = !slowDrive;
                driveMultiplier = slowDrive ? driveMultiplierSlow : driveMultiplierNormal;
                while (gamepad1.a) {}
            }
            babaji.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y * driveMultiplier,
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
                while (gamepad1.left_trigger > 0.2) {}
            }
            if (gamepad1.left_bumper && gamepad1.right_bumper) {
                babaji.droneLauncher.release();
                while (gamepad1.left_bumper && gamepad1.right_bumper) {}
            }
//            if (gamepad1.dpad_down) {
//                babaji.travelToIntake(babaji.getPoseEstimate());
//                while (gamepad1.dpad_down) {}
//            }
//            if (gamepad1.dpad_up) {
//                babaji.travelToOuttake(babaji.getPoseEstimate());
//                while (gamepad1.dpad_up) {}
//            }

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
            if (gamepad2.dpad_up || gamepad2.dpad_left || gamepad2.dpad_right) {
                babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.BACKDROP);
                babaji.outtake.setJonnyRotate(Outtake.JonnyRotatePos.VERTICAL);
                jonnyRotatePosIndex = 1;
                babaji.outtake.setJonnyExtrudePos(Outtake.JonnyExtrudePos.COLLECT);
            }
            if (gamepad2.dpad_down) {
                babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.SHIELD);
                babaji.outtake.setJonnyRotate(Outtake.JonnyRotatePos.VERTICAL);
                jonnyRotatePosIndex = 1;
            }
//            if (gamepad2.dpad_up && !raiseSlides.isAlive()) {
//                // Slides High
////                babaji.outtake.raiseToHeight(Outtake.DepositLevel.HIGH);
//                outtakeLevel = Outtake.DepositLevel.HIGH;
//                raiseSlides.start();
//                while (gamepad2.dpad_up) {}
//            }
//            if (gamepad2.dpad_left && !raiseSlides.isAlive()) {
//                // Slides Mid
////                babaji.outtake.raiseToHeight(Outtake.DepositLevel.MID);
//                outtakeLevel = Outtake.DepositLevel.MID;
//                raiseSlides.start();
//                while (gamepad2.dpad_left) {}
//            } else if (gamepad2.dpad_right && !raiseSlides.isAlive()) {
//                // Slides Low
////                babaji.outtake.raiseToHeight(Outtake.DepositLevel.LOW);
//                outtakeLevel = Outtake.DepositLevel.LOW;
//                raiseSlides.start();
//                while (gamepad2.dpad_right) {}
//            } else if (gamepad2.dpad_down && !raiseSlides.isAlive()) {
//                // Slides Down
////                babaji.outtake.raiseToHeight(Outtake.DepositLevel.GROUND);
//                outtakeLevel = Outtake.DepositLevel.GROUND;
//                raiseSlides.start();
//                while (gamepad2.dpad_down) {}
//            }
            if (Math.abs(gamepad2.right_stick_y) > 0.3) {
                // Slides Micro Adjustment
                babaji.outtake.setSlidesPow(gamepad2.right_stick_y * -1);
            } else {
                babaji.outtake.setSlidesPow(Outtake.restPow);
            }
            if (gamepad2.left_bumper) {
                // Rotate Jonny Clockwise Toggle
                jonnyRotatePosIndex--;
                if (jonnyRotatePosIndex < 0) {
                    jonnyRotatePosIndex = 0;
                }
                babaji.outtake.setJonnyRotate(jonnyRotatePoses[jonnyRotatePosIndex]);
                while (gamepad2.left_bumper) {}
            }
            if (gamepad2.right_bumper) {
                // Rotate Jonny Counter-Clockwise Toggle
                jonnyRotatePosIndex++;
                if (jonnyRotatePosIndex >= jonnyRotatePoses.length) {
                    jonnyRotatePosIndex = jonnyRotatePoses.length-1;
                }
                babaji.outtake.setJonnyRotate(jonnyRotatePoses[jonnyRotatePosIndex]);
                while (gamepad2.right_bumper) {}
            }
            if (gamepad2.left_trigger > 0.2 && gamepad2.right_trigger > 0.2) {
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
                while (gamepad2.left_trigger > 0.2 || gamepad2.right_trigger > 0.2) {}
            } else if (gamepad2.left_trigger > 0.2) {
                // Front Jonny Score
                isJonnyFrontExtrude = !isJonnyFrontExtrude;
                if (isJonnyFrontExtrude) {
                    babaji.outtake.setJonnyFrontExtrudePos(Outtake.JonnyExtrudePos.SCORE);
                } else {
                    babaji.outtake.setJonnyFrontExtrudePos(Outtake.JonnyExtrudePos.COLLECT);
                }
                while (gamepad2.left_trigger > 0.2) {}
            } else if (gamepad2.right_trigger > 0.2) {
                // Back Jonny Score
                isJonnyBackExtrude = !isJonnyBackExtrude;
                if (isJonnyBackExtrude) {
                    babaji.outtake.setJonnyBackExtrudePos(Outtake.JonnyExtrudePos.SCORE);
                } else {
                    babaji.outtake.setJonnyBackExtrudePos(Outtake.JonnyExtrudePos.COLLECT);
                }
                while (gamepad2.right_trigger > 0.2) {}
            }
            if (gamepad2.a) {
                // Auto Ground Intake
                babaji.intake.intakePixel1();
//                collect.start();
//                while (gamepad2.a) {}
//                sleep(500);
            }
//            if (collect.isAlive() && Math.abs(gamepad2.left_stick_y) > 0.5) {
//                collect.interrupt();
//                babaji.intake.setPow(0);
//            }

            babaji.intake.setPow(gamepad2.left_stick_y);

            if (gamepad2.y && !transfer.isAlive()) {
                // Transfer
                babaji.intake.setPow(0.0);
                babaji.intake.raiseIntakeMoving();
                transfer.start();
            }
            if (gamepad2.b) {
                babaji.intake.raiseIntakeMoving();
//                while (gamepad2.b) {}
                sleep(500);
            }

            telemetry.addData("Slow Drive", slowDrive);
            telemetry.addData("Slides Enc Count", babaji.outtake.getEncCount());
            telemetry.addData("Case", currentMode);
            telemetry.update();
        }
//        }
//
//        chassis.interrupt();
//        babaji.setMotorPowers(0,0,0,0);
//        babaji.stop();
    }

    private class Collect extends Thread {
        @Override
        public void run() {
            try {
                babaji.collectGroundMoving();
            }
            catch (InterruptedException e) {}
            this.interrupt();
//            babaji.intake.setPow(0.0);
        }
    }

    private class Transfer extends Thread {
        @Override
        public void run() {
            try {
                babaji.transfer();
            } catch (InterruptedException e) {}
            this.interrupt();
        }
    }

    private class RaiseSlides extends Thread {
        @Override
        public void run() {
            try {
                babaji.outtake.raiseToHeight(outtakeLevel);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }

            this.interrupt();
            babaji.outtake.setSlidesPow(Outtake.restPow);
        }
    }

    private class Chassis extends Thread {
        @Override
        public void run() {
            try {
                while (opModeIsActive()) {
                    babaji.setWeightedDrivePower(
                            new Pose2d(
                                    -gamepad1.left_stick_y * driveMultiplier,
                                    -gamepad1.left_stick_x * driveMultiplier,
                                    -gamepad1.right_stick_x * driveMultiplier
                            )
                    );
                    babaji.update();
                }
            } catch (Exception e) {
            }

            this.interrupt();
        }
    }
}