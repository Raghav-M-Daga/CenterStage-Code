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
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.DetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@Autonomous(preselectTeleOp = "InkOp")
public class AutoRedClosePreload extends LinearOpMode {
    OpenCvCamera camera;
    DetectionPipeline detectionPipeline;
    Babaji babaji;
    public static double leftPosX = -26, leftPosY = 2.5, leftPosHeading = 90, backdropScoreLeftX = -31;
    public static double rightPosX = -15, rightPosY = 10, rightPosHeading = 0, backdropScoreRightX = -20;
    public static double midPosX = -23, midPosY = 4, midPosHeading = 0, backdropScoreMidX = -25.5;
    public static double backdropScoreAngle = 270, backdropScoreY = 38;
    public static double parkX = -40, parkY = 34;
    public static double maxVA = 60;
    public static double lowVA = 30;
    public static double setToOuttakePreDist = 0.1, setToOuttakePreOffset = 0, setToOuttakeDist = 75, setToOuttakeOffset = 0;
//    public static int pos = 1;
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

        // Left Line Trajectory
        TrajectorySequence leftTrajStart = babaji.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .setVelConstraint((v, pose2d, pose2d1, pose2d2) -> lowVA)
                .setAccelConstraint((a, pose2d, pose2d1, pose2d2) -> lowVA)
                .addDisplacementMarker(setToOuttakePreDist, setToOuttakePreOffset, () -> {
                    babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.GROUND);
                })
                .lineToLinearHeading(new Pose2d(leftPosX, leftPosY, Math.toRadians(leftPosHeading)))
                .build();
        TrajectorySequence leftTrajYellow = babaji.trajectorySequenceBuilder(leftTrajStart.end())
                .lineToLinearHeading(new Pose2d(backdropScoreLeftX, backdropScoreY, Math.toRadians(backdropScoreAngle)))
                .build();

//         Mid Line Trajectory
        TrajectorySequence midTrajStart = babaji.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .setVelConstraint((v, pose2d, pose2d1, pose2d2) -> lowVA)
                .setAccelConstraint((a, pose2d, pose2d1, pose2d2) -> lowVA)
                .addDisplacementMarker(setToOuttakePreDist, setToOuttakePreOffset, () -> {
                    babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.GROUND);
                })
                .lineToLinearHeading(new Pose2d(midPosX, midPosY, Math.toRadians(midPosHeading)))
                .build();
        TrajectorySequence midTrajYellow = babaji.trajectorySequenceBuilder(midTrajStart.end())
                .lineToLinearHeading(new Pose2d(backdropScoreMidX, backdropScoreY, Math.toRadians(backdropScoreAngle)))
                .build();
//
//        // Right Line Trajectory
        TrajectorySequence rightTrajStart = babaji.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .setVelConstraint((v, pose2d, pose2d1, pose2d2) -> lowVA)
                .setAccelConstraint((a, pose2d, pose2d1, pose2d2) -> lowVA)
                .addDisplacementMarker(setToOuttakePreDist, setToOuttakePreOffset, () -> {
                    babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.GROUND);
                })
                .lineToLinearHeading(new Pose2d(rightPosX, rightPosY, Math.toRadians(rightPosHeading)))
                .build();
        TrajectorySequence rightTrajYellow = babaji.trajectorySequenceBuilder(rightTrajStart.end())
                .lineToLinearHeading(new Pose2d(backdropScoreRightX, backdropScoreY, Math.toRadians(backdropScoreAngle)))
                .build();

        if (isStopRequested()) return;
        switch (pos) {
            case LEFT:
                babaji.followTrajectorySequence(leftTrajStart);
                babaji.outtake.setJonnyBackExtrudePos(Outtake.JonnyExtrudePos.SCORE);
                sleep(500);
                babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.SHIELD);
                sleep(500);
                babaji.followTrajectorySequence(leftTrajYellow);
                sleep(500);
                babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.BACKDROP);
                sleep(800);
                babaji.outtake.setJonnyFrontExtrudePos(Outtake.JonnyExtrudePos.SCORE);
                sleep(800);
                babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.SHIELD);
                break;
            case MID:
                babaji.followTrajectorySequence(midTrajStart);
                babaji.outtake.setJonnyBackExtrudePos(Outtake.JonnyExtrudePos.SCORE);
                sleep(500);
                babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.BACKDROP);
                sleep(500);
                babaji.followTrajectorySequence(midTrajYellow);
                sleep(800);
                babaji.outtake.setJonnyFrontExtrudePos(Outtake.JonnyExtrudePos.SCORE);
                sleep(800);
                babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.SHIELD);
                break;
            case RIGHT:
                babaji.followTrajectorySequence(rightTrajStart);
                babaji.outtake.setJonnyBackExtrudePos(Outtake.JonnyExtrudePos.SCORE);
                sleep(500);
                babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.BACKDROP);
                sleep(500);
                babaji.followTrajectorySequence(rightTrajYellow);
                sleep(800);
                babaji.outtake.setJonnyFrontExtrudePos(Outtake.JonnyExtrudePos.SCORE);
                sleep(800);
                babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.SHIELD);
                break;
        }
        sleep(500);
        Trajectory park = babaji.trajectoryBuilder(babaji.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(parkX, parkY, Math.toRadians(backdropScoreAngle)))
                .build();

        babaji.followTrajectory(park);


    }
}