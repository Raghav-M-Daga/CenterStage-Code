package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.Babaji;
import org.firstinspires.ftc.teamcode.drive.Outtake;
import org.firstinspires.ftc.teamcode.drive.Sensors;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.DetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
@Autonomous(preselectTeleOp = "InkOp")
public class AutoBlueFarCycle extends LinearOpMode {
    OpenCvCamera camera;
    DetectionPipeline detectionPipeline;
    Sensors sensor;

    public static double rightPosX = -20, rightPosY = 10, rightPosAngle = 0, backdropScoreRightX = 36, backdropScoreRightY = -34;
    public static double leftPosX = -29, leftPosY = 2, leftPosAngle = 90, backdropScoreLeftX = -23;
    public static double midPosX = -29, midPosY = 4, midPosAngle = 0, backdropScoreMidX = -30;
    public static double intakeX = -27, intakeY = 18, intakeAngle = 90;
    public static double straightSetX = 3.5, straightSetY = 4, straightReturnY = -15;
    public static double straightEndY = 55, straightInterpol = -90;
    public static double backdropScoreAngle = 90, backdropScoreInterpol = -90, backDropScoreY = 86;
    public static double outtakeToGroundDist = 0.07, outtakeToGroundOffset = 0, preloadToBackdropDist = 0.2, PreloadToBackdropDist2 = 0.8;
    public static double parkX = -5, parkY = -80;
    public static double maxVA = 90;
    public static double lowVA = 30;
    public static double strafeDist = 5.5;
//    public static int pos = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        Babaji babaji = new Babaji(hardwareMap);
        sensor = new Sensors(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "backCamera"), cameraMonitorViewId);
        detectionPipeline = new DetectionPipeline();

        camera.setPipeline(detectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(1280,960, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        FtcDashboard.getInstance().startCameraStream(camera, 0);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.setMsTransmissionInterval(50);

        while (!isStarted() && !isStopRequested())
        {
            DetectionPipeline.POSITION pos = detectionPipeline.getPos();
            telemetry.addData("Current Pos", pos.toString());
            telemetry.addData("LEFT", detectionPipeline.getLeftBoxVal());
            telemetry.addData("MID", detectionPipeline.getMidBoxVal());
            telemetry.addData("RIGHT", detectionPipeline.getRightBoxVal());

            telemetry.update();
            sleep(20);
        }

        waitForStart();

        DetectionPipeline.POSITION pos = detectionPipeline.getPos();

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        babaji.setPoseEstimate(startPose);

        // Right Line Trajectory
        TrajectorySequence rightTrajPreload = babaji.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .setVelConstraint((v, pose2d, pose2d1, pose2d2) -> lowVA)
                .setAccelConstraint((a, pose2d, pose2d1, pose2d2) -> lowVA)
                .addDisplacementMarker(outtakeToGroundDist, outtakeToGroundOffset, () -> {
                    babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.GROUND);
                })
                .lineToLinearHeading(new Pose2d(rightPosX, rightPosY, Math.toRadians(rightPosAngle)))
                .build();

        TrajectorySequence rightTrajIntake = babaji.trajectorySequenceBuilder(rightTrajPreload.end())
                .lineToLinearHeading(new Pose2d(intakeX, intakeY, Math.toRadians(intakeAngle)))
                .addDisplacementMarker(() -> {
                    babaji.outtake.setJonnyExtrudePos(Outtake.JonnyExtrudePos.SCORE);
                    babaji.creep();
                })
                .build();

        // Middle Line Trajectory

        TrajectorySequence midTrajPreload = babaji.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .setVelConstraint((v, pose2d, pose2d1, pose2d2) -> lowVA)
                .setAccelConstraint((a, pose2d, pose2d1, pose2d2) -> lowVA)
                .addDisplacementMarker(outtakeToGroundDist, outtakeToGroundOffset, () -> {
                    babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.GROUND);
                })
                .lineToLinearHeading(new Pose2d(midPosX, midPosY, Math.toRadians(midPosAngle)))
                .build();

        TrajectorySequence midTrajIntake = babaji.trajectorySequenceBuilder(midTrajPreload.end())
                .setReversed(false)
                .setVelConstraint((v, pose2d, pose2d1, pose2d2) -> lowVA)
                .setAccelConstraint((a, pose2d, pose2d1, pose2d2) -> lowVA)
                .lineToLinearHeading(new Pose2d(intakeX, intakeY, Math.toRadians(intakeAngle)))
                .addDisplacementMarker(() -> {
                    babaji.outtake.setJonnyExtrudePos(Outtake.JonnyExtrudePos.SCORE);
                    babaji.creep();
                })
                .build();

        // Left Line Trajectory
        TrajectorySequence leftTrajPreload = babaji.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .setVelConstraint((v, pose2d, pose2d1, pose2d2) -> lowVA)
                .setAccelConstraint((a, pose2d, pose2d1, pose2d2) -> lowVA)
                .addDisplacementMarker(outtakeToGroundDist, outtakeToGroundOffset, () -> {
                    babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.GROUND);
                })
                .lineToLinearHeading(new Pose2d(leftPosX, leftPosY, Math.toRadians(leftPosAngle)))
                .build();

        TrajectorySequence leftTrajIntake = babaji.trajectorySequenceBuilder(leftTrajPreload.end())
                .lineToLinearHeading(new Pose2d(intakeX, intakeY, Math.toRadians(intakeAngle)))
                .addDisplacementMarker(() -> {
                    babaji.outtake.setJonnyExtrudePos(Outtake.JonnyExtrudePos.SCORE);
                    babaji.creep();
                })
                .build();

        if (isStopRequested()) return;
        if (pos == DetectionPipeline.POSITION.LEFT) {
            babaji.followTrajectorySequence(leftTrajPreload);
            babaji.outtake.setJonnyBackExtrudePos(Outtake.JonnyExtrudePos.SCORE);
            sleep(400);
            babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.SHIELD);
            babaji.outtake.setActuatorRotate(Outtake.ActuatorRotatePos.GROUND);
            sleep(200);
            babaji.followTrajectorySequence(leftTrajIntake);
            babaji.outtake.setActuatorRotate(Outtake.ActuatorRotatePos.SHIELD);
            sleep(300);
            babaji.intake.intakePixel2();
            sleep(400);
            Trajectory topple = babaji.trajectoryBuilder(babaji.getPoseEstimate())
                    .strafeLeft(strafeDist)
                    .build();
            babaji.followTrajectory(topple);
            sleep(500);
            babaji.intake.collectPixel1(babaji);
            babaji.intake.raiseIntake();
            TrajectorySequence depositWhiteLeft = babaji.trajectorySequenceBuilder(babaji.getPoseEstimate())
                    .setReversed(true)
                    .setVelConstraint((v, pose2d, pose2d1, pose2d2) -> lowVA)
                    .setAccelConstraint((a, pose2d, pose2d1, pose2d2) -> lowVA)
                    .lineToLinearHeading(new Pose2d(straightSetX, straightSetY, Math.toRadians(backdropScoreAngle)))
                    .setVelConstraint((v, pose2d, pose2d1, pose2d2) -> maxVA)
                    .setAccelConstraint((a, pose2d, pose2d1, pose2d2) -> maxVA)
                    .splineToLinearHeading(new Pose2d(straightSetX, straightEndY, Math.toRadians(backdropScoreAngle)), Math.toRadians(straightInterpol))
                    .setVelConstraint((v, pose2d, pose2d1, pose2d2) -> lowVA)
                    .setAccelConstraint((a, pose2d, pose2d1, pose2d2) -> lowVA)
                    .splineToLinearHeading(new Pose2d(backdropScoreLeftX, backDropScoreY, Math.toRadians(backdropScoreAngle)), Math.toRadians(backdropScoreInterpol))
                    .build();
            babaji.followTrajectorySequence(depositWhiteLeft);
            babaji.intake.setPow(0.5);
            babaji.transfer();
            babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.BACKDROP);
            babaji.intake.setPow(0);
            sleep(500);
            babaji.outtake.setJonnyRotate(Outtake.JonnyRotatePos.HORIZONTAL);
            sleep(400);
            babaji.outtake.setJonnyExtrudePos(Outtake.JonnyExtrudePos.SCORE);
            sleep(500);
            Trajectory park = babaji.trajectoryBuilder(depositWhiteLeft.end())
                    .lineToLinearHeading(new Pose2d(parkX, parkY, Math.toRadians(backdropScoreAngle)))
                    .build();
            babaji.followTrajectory(park);
            babaji.outtake.setJonnyRotate(Outtake.JonnyRotatePos.VERTICAL);
            sleep(500);
            babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.SHIELD);
            sleep(500);
        } else if (pos == DetectionPipeline.POSITION.MID) {
            babaji.followTrajectorySequence(midTrajPreload);
            babaji.outtake.setJonnyBackExtrudePos(Outtake.JonnyExtrudePos.SCORE);
            sleep(400);
            babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.SHIELD);
            babaji.outtake.setActuatorRotate(Outtake.ActuatorRotatePos.GROUND);
            sleep(200);
            babaji.followTrajectorySequence(midTrajIntake);
            babaji.outtake.setActuatorRotate(Outtake.ActuatorRotatePos.SHIELD);
            sleep(300);
            babaji.intake.intakePixel2();
            sleep(400);
            Trajectory topple = babaji.trajectoryBuilder(babaji.getPoseEstimate())
                    .strafeLeft(strafeDist)
                    .build();
            babaji.followTrajectory(topple);
            sleep(500);

            babaji.intake.collectPixel1(babaji);
            babaji.intake.raiseIntake();
            TrajectorySequence depositWhiteMid = babaji.trajectorySequenceBuilder(topple.end())
                    .setReversed(true)
                    .setVelConstraint((v, pose2d, pose2d1, pose2d2) -> lowVA)
                    .setAccelConstraint((a, pose2d, pose2d1, pose2d2) -> lowVA)
                    .lineToLinearHeading(new Pose2d(straightSetX, straightSetY, Math.toRadians(backdropScoreAngle)))
                    .setVelConstraint((v, pose2d, pose2d1, pose2d2) -> maxVA)
                    .setAccelConstraint((a, pose2d, pose2d1, pose2d2) -> maxVA)
                    .splineToLinearHeading(new Pose2d(straightSetX, straightEndY, Math.toRadians(backdropScoreAngle)), Math.toRadians(straightInterpol))
                    .setVelConstraint((v, pose2d, pose2d1, pose2d2) -> lowVA)
                    .setAccelConstraint((a, pose2d, pose2d1, pose2d2) -> lowVA)
                    .splineToLinearHeading(new Pose2d(backdropScoreMidX, backDropScoreY, Math.toRadians(backdropScoreAngle)), Math.toRadians(backdropScoreInterpol))
                    .build();
            babaji.followTrajectorySequence(depositWhiteMid);
            babaji.intake.setPow(0.5);
            babaji.transfer();
            babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.BACKDROP);
            babaji.intake.setPow(0);
            sleep(500);
            babaji.outtake.setJonnyRotate(Outtake.JonnyRotatePos.HORIZONTAL);
            sleep(400);
            babaji.outtake.setJonnyExtrudePos(Outtake.JonnyExtrudePos.SCORE);
            sleep(500);
            Trajectory park = babaji.trajectoryBuilder(depositWhiteMid.end())
                    .lineToLinearHeading(new Pose2d(parkX, parkY, Math.toRadians(backdropScoreAngle)))
                    .build();
            babaji.followTrajectory(park);
            babaji.outtake.setJonnyRotate(Outtake.JonnyRotatePos.VERTICAL);
            sleep(500);
            babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.SHIELD);
            sleep(500);
        } else if (pos == DetectionPipeline.POSITION.RIGHT) {
            babaji.followTrajectorySequence(rightTrajPreload);
            babaji.outtake.setJonnyBackExtrudePos(Outtake.JonnyExtrudePos.SCORE);
            sleep(400);
            babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.SHIELD);
            babaji.outtake.setActuatorRotate(Outtake.ActuatorRotatePos.GROUND);
            sleep(200);
            babaji.followTrajectorySequence(rightTrajIntake);
            babaji.outtake.setActuatorRotate(Outtake.ActuatorRotatePos.SHIELD);
            sleep(300);
            babaji.intake.intakePixel2();
            sleep(400);
            Trajectory topple = babaji.trajectoryBuilder(babaji.getPoseEstimate())
                    .strafeLeft(strafeDist)
                    .build();
            babaji.followTrajectory(topple);
            sleep(500);

            babaji.intake.collectPixel1(babaji);
            babaji.intake.raiseIntake();
            TrajectorySequence depositWhiteRight = babaji.trajectorySequenceBuilder(topple.end())
                    .setReversed(true)
                    .setVelConstraint((v, pose2d, pose2d1, pose2d2) -> lowVA)
                    .setAccelConstraint((a, pose2d, pose2d1, pose2d2) -> lowVA)
                    .lineToLinearHeading(new Pose2d(straightSetX, straightSetY, Math.toRadians(backdropScoreAngle)))
                    .setVelConstraint((v, pose2d, pose2d1, pose2d2) -> maxVA)
                    .setAccelConstraint((a, pose2d, pose2d1, pose2d2) -> maxVA)
                    .splineToLinearHeading(new Pose2d(straightSetX, straightEndY, Math.toRadians(backdropScoreAngle)), Math.toRadians(straightInterpol))
                    .setVelConstraint((v, pose2d, pose2d1, pose2d2) -> lowVA)
                    .setAccelConstraint((a, pose2d, pose2d1, pose2d2) -> lowVA)
                    .splineToLinearHeading(new Pose2d(backdropScoreRightX, backDropScoreY, Math.toRadians(backdropScoreAngle)), Math.toRadians(backdropScoreInterpol))
                    .build();
            babaji.followTrajectorySequence(depositWhiteRight);
            babaji.intake.setPow(0.5);
            babaji.transfer();
            babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.BACKDROP);
            babaji.intake.setPow(0);
            sleep(500);
            babaji.outtake.setJonnyRotate(Outtake.JonnyRotatePos.HORIZONTAL);
            sleep(400);
            babaji.outtake.setJonnyExtrudePos(Outtake.JonnyExtrudePos.SCORE);
            sleep(500);
            Trajectory park = babaji.trajectoryBuilder(depositWhiteRight.end())
                    .lineToLinearHeading(new Pose2d(parkX, parkY, Math.toRadians(backdropScoreAngle)))
                    .build();
            babaji.followTrajectory(park);
            babaji.outtake.setJonnyRotate(Outtake.JonnyRotatePos.VERTICAL);
            sleep(500);
            babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.SHIELD);
            sleep(500);
        }
    }
}