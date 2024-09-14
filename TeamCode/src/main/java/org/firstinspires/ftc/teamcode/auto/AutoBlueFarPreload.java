package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
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

@Config
@Autonomous(preselectTeleOp = "InkOp")
public class AutoBlueFarPreload extends LinearOpMode {
    OpenCvCamera camera;
    DetectionPipeline detectionPipeline;
    Sensors sensor;

    // old
    public static double rightPosX = -13.5, rightPosY = 10.5, rightPosAngle = 0, backdropScoreRightX = -30.5;
    public static double leftPosX = -26.5, leftPosY = 2.5, leftPosAngle = 90, backdropScoreLeftX = -19.5;
    public static double midPosX = -23, midPosY = 4, midPosAngle = 0, backdropScoreMidX = -25;
    public static double straightSetX = -2.5, straightSetY = 3;
//    straightReturnY = -15;
    public static double straightEndY = -55, straightInterpol = 270;
    public static double backdropScoreAngle = 90, backdropScoreInterpol = 270, backDropScoreY = -87.5;
    public static double parkX = -5, parkY = -78;
    public static double maxVA = 60;
    public static double lowVA = 30;
    public static double outtakeToGroundDist = 0.07, outtakeToGroundOffset = 0, preloadToBackdropDist = 0.2, PreloadToBackdropDist2 = 0.8;


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

        // Updated vision pose
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
//                .addDisplacementMarker(() -> {
//                    babaji.outtake.setJonnyBackExtrudePos(Outtake.JonnyExtrudePos.SCORE);
//                })
                .build();

        TrajectorySequence rightTrajPreloadToBackdrop = babaji.trajectorySequenceBuilder(rightTrajPreload.end())
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(straightSetX, straightSetY, Math.toRadians(backdropScoreAngle)))
                .waitSeconds(0.5)
                .setVelConstraint((v, pose2d, pose2d1, pose2d2) -> maxVA)
                .setAccelConstraint((a, pose2d, pose2d1, pose2d2) -> maxVA)
                .splineToLinearHeading(new Pose2d(straightSetX, straightEndY, Math.toRadians(backdropScoreAngle)), Math.toRadians(straightInterpol))
                .setVelConstraint((v, pose2d, pose2d1, pose2d2) -> lowVA)
                .setAccelConstraint((a, pose2d, pose2d1, pose2d2) -> lowVA)
                .splineToLinearHeading(new Pose2d(backdropScoreRightX, backDropScoreY, Math.toRadians(backdropScoreAngle)), Math.toRadians(backdropScoreInterpol))
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

        TrajectorySequence midTrajPreloadToBackdrop = babaji.trajectorySequenceBuilder(midTrajPreload.end())
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(straightSetX, straightSetY, Math.toRadians(backdropScoreAngle)))
                .waitSeconds(0.5)
                .setVelConstraint((v, pose2d, pose2d1, pose2d2) -> maxVA)
                .setAccelConstraint((a, pose2d, pose2d1, pose2d2) -> maxVA)
                .splineToLinearHeading(new Pose2d(straightSetX, straightEndY, Math.toRadians(backdropScoreAngle)), Math.toRadians(straightInterpol))
                .setVelConstraint((v, pose2d, pose2d1, pose2d2) -> lowVA)
                .setAccelConstraint((a, pose2d, pose2d1, pose2d2) -> lowVA)
                .splineToLinearHeading(new Pose2d(backdropScoreMidX, backDropScoreY, Math.toRadians(backdropScoreAngle)), Math.toRadians(backdropScoreInterpol))
                .build();

        // Left Line Trajectory
        TrajectorySequence leftTrajPreload = babaji.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .setVelConstraint((v, pose2d, pose2d1, pose2d2) -> lowVA)
                .setAccelConstraint((a, pose2d, pose2d1, pose2d2) -> lowVA)
                .addDisplacementMarker(outtakeToGroundDist, outtakeToGroundOffset, () -> {
                    babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.GROUND);
                })
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(leftPosX, leftPosY, Math.toRadians(leftPosAngle)))
                .build();

        TrajectorySequence leftTrajPreloadToBackdrop = babaji.trajectorySequenceBuilder(leftTrajPreload.end())
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(straightSetX, straightSetY, Math.toRadians(backdropScoreAngle)))
                .waitSeconds(0.5)
                .setVelConstraint((v, pose2d, pose2d1, pose2d2) -> maxVA)
                .setAccelConstraint((a, pose2d, pose2d1, pose2d2) -> maxVA)
                .splineToLinearHeading(new Pose2d(straightSetX, straightEndY, Math.toRadians(backdropScoreAngle)), Math.toRadians(straightInterpol))
                .setVelConstraint((v, pose2d, pose2d1, pose2d2) -> lowVA)
                .setAccelConstraint((a, pose2d, pose2d1, pose2d2) -> lowVA)
                .splineToLinearHeading(new Pose2d(backdropScoreLeftX, backDropScoreY, Math.toRadians(backdropScoreAngle)), Math.toRadians(backdropScoreInterpol))
                .build();

        if (isStopRequested()) return;
        sleep(1000);
        switch (pos) {
            case LEFT:
                babaji.followTrajectorySequence(leftTrajPreload);
                babaji.outtake.setJonnyBackExtrudePos(Outtake.JonnyExtrudePos.SCORE);
                sleep(1000);
                babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.SHIELD);
                sleep(500);
                babaji.followTrajectorySequence(leftTrajPreloadToBackdrop);
                babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.BACKDROP);
                sleep(800);
                babaji.outtake.setJonnyExtrudePos(Outtake.JonnyExtrudePos.SCORE);
                sleep(500);
                break;

            case MID:
                babaji.followTrajectorySequence(midTrajPreload);
                babaji.outtake.setJonnyBackExtrudePos(Outtake.JonnyExtrudePos.SCORE);
                sleep(1000);
                babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.SHIELD);
                sleep(500);
                babaji.followTrajectorySequence(midTrajPreloadToBackdrop);
                babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.BACKDROP);
                sleep(800);
                babaji.outtake.setJonnyExtrudePos(Outtake.JonnyExtrudePos.SCORE);
                sleep(500);
                break;

            case RIGHT:
                babaji.followTrajectorySequence(rightTrajPreload);
                babaji.outtake.setJonnyBackExtrudePos(Outtake.JonnyExtrudePos.SCORE);
                sleep(1000);
                babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.SHIELD);
                sleep(500);
                babaji.followTrajectorySequence(rightTrajPreloadToBackdrop);
                babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.BACKDROP);
                sleep(800);
                babaji.outtake.setJonnyExtrudePos(Outtake.JonnyExtrudePos.SCORE);
                sleep(500);
                break;
        }
        babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.SHIELD);
        sleep(500);
        Trajectory park = babaji.trajectoryBuilder(babaji.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(parkX, parkY, Math.toRadians(backdropScoreAngle)))
                .build();
        babaji.followTrajectory(park);
    }
}