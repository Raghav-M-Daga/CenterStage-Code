package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.Babaji;
import org.firstinspires.ftc.teamcode.drive.Intake;
import org.firstinspires.ftc.teamcode.drive.Outtake;
import org.firstinspires.ftc.teamcode.rrpathing.DriveConstants;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
@Autonomous(preselectTeleOp = "InkOp")
public class AutoRedCloseCycle extends LinearOpMode {
    OpenCvWebcam camera;
    Babaji babaji;
    public static double leftPosX = -25, leftPosY = -1.5, leftPosInterpol = 225, leftPosHeading = 270, backdropScoreLeftX = -34, backdropScoreLeftY = 36.5;
    public static double rightPosX = -28, rightPosY = 22, rightPosHeading = 270, backdropScoreRightX = -20, backdropScoreRightY = 36.5;
    public static double midPosX = -26.5, midPosY = 5.3, midPosHeading = 270, backdropScoreMidX = -27, backdropScoreMidY = 36.5;
    public static double intakeX = -55, intakeFinalX = -48, intakeSplineY = 0, intakeStraightY1 = -63, intakeStraightY2 = -60, intakeAngle = 270;
    public static double backdropScoreAngle = 270, backdropScoreAngleInterpolation = 90, backdropScoreWhiteX = -20;
    public static double parkX = -4, parkY = 34;
    public static double maxVA = 60;
    public static double lowVA = 30;
    public static double setToOuttakePreDist = 0.1, setToOuttakePreOffset = 0, setToOuttakeDist = 75, setToOuttakeOffset = 0;
    public static int pos = 1;
    public enum AutoLine {
        LEFT,
        MID,
        RIGHT,
    }

    @Override
    public void runOpMode() throws InterruptedException {

//        DropPurple dropPurple = new DropPurple();
        babaji = new Babaji(hardwareMap);
//        Sensors sensors = new Sensors(hardwareMap);
        FtcDashboard.getInstance().startCameraStream(camera, 0);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.setMsTransmissionInterval(50);


        waitForStart();

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        babaji.setPoseEstimate(startPose);

        // Left Line Trajectory
        TrajectorySequence leftTrajStart = babaji.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .setVelConstraint((v, pose2d, pose2d1, pose2d2) -> lowVA)
                .setAccelConstraint((a, pose2d, pose2d1, pose2d2) -> lowVA)
                .addDisplacementMarker(setToOuttakePreDist, setToOuttakePreOffset, () -> {
                    babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.BACKDROP);
                })
                .lineToLinearHeading(new Pose2d(leftPosX, leftPosY, Math.toRadians(leftPosHeading)))
                .build();
        babaji.intake.setPow(0.5);
        sleep(1000);
        babaji.intake.setPow(0);
        TrajectorySequence leftTrajYellow = babaji.trajectorySequenceBuilder(leftTrajStart.end())
                .lineToLinearHeading(new Pose2d(backdropScoreLeftX, backdropScoreLeftY, Math.toRadians(backdropScoreAngle)))
                .build();

        babaji.outtake.setJonnyExtrudePos(Outtake.JonnyExtrudePos.SCORE);
        sleep(500);
        babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.SHIELD);
        TrajectorySequence intakeWhiteLeft = babaji.trajectorySequenceBuilder(leftTrajYellow.end())
                .setReversed(false)
                .setVelConstraint((v, pose2d, pose2d1, pose2d2) -> lowVA)
                .setAccelConstraint((a, pose2d, pose2d1, pose2d2) -> lowVA)
                .splineToLinearHeading(new Pose2d(intakeX, intakeSplineY, Math.toRadians(intakeAngle)), Math.toRadians(intakeAngle))
                .setVelConstraint((v, pose2d, pose2d1, pose2d2) -> maxVA)
                .setAccelConstraint((a, pose2d, pose2d1, pose2d2) -> maxVA)
                .splineToLinearHeading(new Pose2d(intakeFinalX, intakeStraightY1, Math.toRadians(intakeAngle)), Math.toRadians(intakeAngle))
                .addDisplacementMarker(() -> {
                    babaji.creep();
                })
                .build();

//         Mid Line Trajectory
        TrajectorySequence midTrajStart = babaji.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .setVelConstraint((v, pose2d, pose2d1, pose2d2) -> lowVA)
                .setAccelConstraint((a, pose2d, pose2d1, pose2d2) -> lowVA)
                .addDisplacementMarker(setToOuttakePreDist, setToOuttakePreOffset, () -> {
                    babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.BACKDROP);
                })
                .lineToLinearHeading(new Pose2d(midPosX, midPosY, Math.toRadians(midPosHeading)))
                .build();
        babaji.intake.setPow(0.5);
        sleep(1000);
        babaji.intake.setPow(0);
        TrajectorySequence midTrajYellow = babaji.trajectorySequenceBuilder(midTrajStart.end())
                .lineToLinearHeading(new Pose2d(backdropScoreMidX, backdropScoreMidY, Math.toRadians(backdropScoreAngle)))
                .build();

        babaji.outtake.setJonnyExtrudePos(Outtake.JonnyExtrudePos.SCORE);
        sleep(500);
        babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.SHIELD);
        TrajectorySequence intakeWhiteMid = babaji.trajectorySequenceBuilder(midTrajYellow.end())
                .setReversed(false)
                .setVelConstraint((v, pose2d, pose2d1, pose2d2) -> lowVA)
                .setAccelConstraint((a, pose2d, pose2d1, pose2d2) -> lowVA)
                .splineToLinearHeading(new Pose2d(intakeX, intakeSplineY, Math.toRadians(intakeAngle)), Math.toRadians(intakeAngle))
                .setVelConstraint((v, pose2d, pose2d1, pose2d2) -> maxVA)
                .setAccelConstraint((a, pose2d, pose2d1, pose2d2) -> maxVA)
                .splineToLinearHeading(new Pose2d(intakeFinalX, intakeStraightY1, Math.toRadians(intakeAngle)), Math.toRadians(intakeAngle))
                .addDisplacementMarker(() -> {
                    babaji.creep();
                })
                .build();
//
//        // Right Line Trajectory
        TrajectorySequence rightTrajStart = babaji.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .setVelConstraint((v, pose2d, pose2d1, pose2d2) -> lowVA)
                .setAccelConstraint((a, pose2d, pose2d1, pose2d2) -> lowVA)
                .addDisplacementMarker(setToOuttakePreDist, setToOuttakePreOffset, () -> {
                    babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.BACKDROP);
                })
                .lineToLinearHeading(new Pose2d(rightPosX, rightPosY, Math.toRadians(rightPosHeading)))
                .build();
        babaji.intake.setPow(0.5);
        sleep(1000);
        babaji.intake.setPow(0);
        TrajectorySequence rightTrajYellow = babaji.trajectorySequenceBuilder(rightTrajStart.end())
                .lineToLinearHeading(new Pose2d(backdropScoreRightX, backdropScoreRightY, Math.toRadians(backdropScoreAngle)))
                .build();

        babaji.outtake.setJonnyExtrudePos(Outtake.JonnyExtrudePos.SCORE);
        sleep(500);
        babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.SHIELD);
        TrajectorySequence intakeWhiteRight = babaji.trajectorySequenceBuilder(rightTrajYellow.end())
                .setReversed(false)
                .setVelConstraint((v, pose2d, pose2d1, pose2d2) -> lowVA)
                .setAccelConstraint((a, pose2d, pose2d1, pose2d2) -> lowVA)
                .splineToLinearHeading(new Pose2d(intakeX, intakeSplineY, Math.toRadians(intakeAngle)), Math.toRadians(intakeAngle))
                .setVelConstraint((v, pose2d, pose2d1, pose2d2) -> maxVA)
                .setAccelConstraint((a, pose2d, pose2d1, pose2d2) -> maxVA)
                .splineToLinearHeading(new Pose2d(intakeFinalX, intakeStraightY1, Math.toRadians(intakeAngle)), Math.toRadians(intakeAngle))
                .addDisplacementMarker(() -> {
                    babaji.creep();
                })
                .build();

        if (isStopRequested()) return;
        switch (pos) {
            case 1:
                babaji.followTrajectorySequence(leftTrajStart);
                break;
            case 2:
                babaji.followTrajectorySequence(midTrajStart);
                break;
            case 3:
                babaji.followTrajectorySequence(rightTrajStart);
                break;
        }
        babaji.intake.collectTopPixelsAuto(babaji);
        babaji.intake.raiseIntake();

        TrajectorySequence depositWhite = babaji.trajectorySequenceBuilder(babaji.getPoseEstimate())
                .setReversed(true)
                .setVelConstraint((v, pose2d, pose2d1, pose2d2) -> maxVA)
                .setAccelConstraint((a, pose2d, pose2d1, pose2d2) -> maxVA)
                .splineToLinearHeading(new Pose2d(intakeX, intakeSplineY, Math.toRadians(backdropScoreAngle)), Math.toRadians(backdropScoreAngleInterpolation))
                .setVelConstraint((v, pose2d, pose2d1, pose2d2) -> lowVA)
                .setAccelConstraint((a, pose2d, pose2d1, pose2d2) -> lowVA)
                .splineToLinearHeading(new Pose2d(backdropScoreWhiteX, backdropScoreLeftY, Math.toRadians(backdropScoreAngle)), Math.toRadians(backdropScoreAngleInterpolation))
                .build();
        babaji.followTrajectorySequence(depositWhite);
        sleep(500);
        babaji.intake.setPow(0.5);
        babaji.outtake.setJonnyExtrudePos(Outtake.JonnyExtrudePos.COLLECT);
        sleep(700);
        babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.INTAKE);
        sleep(700);
        babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.BACKDROP);
        babaji.intake.setPow(0);
        sleep(500);
        babaji.outtake.setJonnyRotate(Outtake.JonnyRotatePos.HORIZONTAL);
        sleep(400);
        babaji.outtake.setJonnyExtrudePos(Outtake.JonnyExtrudePos.SCORE);
        sleep(500);
        babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.SHIELD);
        sleep(500);
        Trajectory park = babaji.trajectoryBuilder(depositWhite.end())
                .lineToLinearHeading(new Pose2d(parkX, parkY, backdropScoreAngle))
                .build();

        babaji.followTrajectory(park);

//        TrajectorySequence intake = babaji.trajectorySequenceBuilder(depositWhite.end())
//                .setReversed(false)
//                .setVelConstraint((v, pose2d, pose2d1, pose2d2) -> lowVA)
//                .setAccelConstraint((a, pose2d, pose2d1, pose2d2) -> lowVA)
//                .splineToLinearHeading(new Pose2d(intakeX, intakeSplineY, Math.toRadians(intakeAngle)), Math.toRadians(intakeAngle))
//                .setVelConstraint((v, pose2d, pose2d1, pose2d2) -> maxVA)
//                .setAccelConstraint((a, pose2d, pose2d1, pose2d2) -> maxVA)
//                .splineToLinearHeading(new Pose2d(intakeFinalX, intakeStraightY1, Math.toRadians(intakeAngle)), Math.toRadians(intakeAngle))
//                .addDisplacementMarker(() -> {
//                    babaji.creep();
//                })
//                .build();
//        babaji.followTrajectorySequence(intake);
//        babaji.intake.collectLowerPixelsAuto(babaji);
//        babaji.intake.raiseIntake();
//        TrajectorySequence depositWhite2 = babaji.trajectorySequenceBuilder(intake.end())
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(intakeX, intakeSplineY, Math.toRadians(backdropScoreAngle)), Math.toRadians(backdropScoreAngleInterpolation))
//                .addDisplacementMarker(setToOuttakeDist, () -> {
//                    babaji.outtake.setJonnyExtrudePos(Outtake.JonnyExtrudePos.COLLECT);
//                    babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.INTAKE);
//                    babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.BACKDROP);
//                })
//                .setVelConstraint((v, pose2d, pose2d1, pose2d2) -> lowVA)
//                .setAccelConstraint((a, pose2d, pose2d1, pose2d2) -> lowVA)
//                .splineToLinearHeading(new Pose2d(backdropScoreWhiteX, backdropScoreLeftY, Math.toRadians(backdropScoreAngle)), Math.toRadians(backdropScoreAngleInterpolation))
//                .addDisplacementMarker(() -> {
//                    babaji.outtake.setJonnyExtrudePos(Outtake.JonnyExtrudePos.SCORE);
//                })
//                .build();
//        babaji.followTrajectorySequence(depositWhite2);
//        sleep(500);
//        babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.SHIELD);

    }
}