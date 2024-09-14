package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.Babaji;
import org.firstinspires.ftc.teamcode.rrpathing.DriveConstants;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.DetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
@Autonomous(preselectTeleOp = "InkOp")
public class AutoBlueCloseCycle extends LinearOpMode {
    OpenCvCamera camera;
    DetectionPipeline detectionPipeline;

    public static double rightPosX = 25, rightPosY = -4, rightPosAngle = -55, backdropScoreRightX = 20, backdropScoreRightY = 34;
    public static double midPosX = 23.5, midPosY = 5.3, backdropScoreMidX = 27, backdropScoreMidY = 35.5;
    public static double leftPosX = 25, leftPosY = 24, leftPosAngle = -55, backdropScoreLeftX = 18, backdropScoreLeftY = 35.5;
    public static double intakeX = 52, intakeFinalX = 52, intakeSplineY = 0, intakeStraightY1 = -62, intakeStraightY2 = -70, intakeAngle = -90;
    public static double backdropScoreAngle = -90, backdropScoreAngleInterpolation = 90, backdropScoreWhiteX = 25;
    public static double parkX = 43, parkY = 34;
    public static double maxVA = 80;
    public static double lowVA = 30;

    @Override
    public void runOpMode() throws InterruptedException {
        Babaji babaji = new Babaji(hardwareMap);

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
        Trajectory rightTrajPreload = babaji.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(rightPosX, rightPosY, Math.toRadians(rightPosAngle)))
                .build();
        Trajectory rightbackdropPreloadTraj = babaji.trajectoryBuilder(rightTrajPreload.end())
                .lineToLinearHeading(new Pose2d(backdropScoreRightX, backdropScoreRightY, Math.toRadians(backdropScoreAngle)))
                .build();

        // Mid Line Trajectory
        Trajectory midTrajPreload = babaji.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(midPosX, midPosY, Math.toRadians(0)))
                .build();
        Trajectory midbackdropPreloadTraj = babaji.trajectoryBuilder(midTrajPreload.end())
                .lineToLinearHeading(new Pose2d(backdropScoreMidX, backdropScoreMidY, Math.toRadians(backdropScoreAngle)))
                .build();

        // Left Line Trajectory
        Trajectory leftTrajPreload = babaji.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(leftPosX, leftPosY, Math.toRadians(leftPosAngle)))
                .build();
        Trajectory leftbackdropPreloadTraj = babaji.trajectoryBuilder(leftTrajPreload.end())
                .lineToLinearHeading(new Pose2d(backdropScoreLeftX, backdropScoreLeftY, Math.toRadians(backdropScoreAngle)))
                .build();


        TrajectorySequence intake = babaji.trajectorySequenceBuilder(rightbackdropPreloadTraj.end())
                .setReversed(false)
                .setVelConstraint((v, pose2d, pose2d1, pose2d2) -> lowVA)
                .setAccelConstraint((a, pose2d, pose2d1, pose2d2) -> lowVA)
                .splineToLinearHeading(new Pose2d(intakeX, intakeSplineY, Math.toRadians(intakeAngle)), Math.toRadians(intakeAngle))
                .setVelConstraint((v, pose2d, pose2d1, pose2d2) -> maxVA)
                .setAccelConstraint((a, pose2d, pose2d1, pose2d2) -> maxVA)
                .splineToLinearHeading(new Pose2d(intakeFinalX, intakeStraightY1, Math.toRadians(intakeAngle)), Math.toRadians(intakeAngle))
                .setVelConstraint((v, pose2d, pose2d1, pose2d2) -> lowVA)
                .setAccelConstraint((a, pose2d, pose2d1, pose2d2) -> lowVA)
                .splineToLinearHeading(new Pose2d(intakeFinalX, intakeStraightY2, Math.toRadians(intakeAngle)), Math.toRadians(intakeAngle))
                .build();

        TrajectorySequence depositWhite = babaji.trajectorySequenceBuilder(intake.end())
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(intakeX, intakeSplineY, Math.toRadians(backdropScoreAngle)), Math.toRadians(backdropScoreAngleInterpolation))
                .setVelConstraint((v, pose2d, pose2d1, pose2d2) -> lowVA)
                .setAccelConstraint((a, pose2d, pose2d1, pose2d2) -> lowVA)
                // TODO: Fix BackdropScoreRightY to something either universal for all trajs or create deposit white for each line
                .splineToLinearHeading(new Pose2d(backdropScoreWhiteX, backdropScoreRightY, Math.toRadians(backdropScoreAngle)), Math.toRadians(backdropScoreAngleInterpolation))
                .build();


        if (isStopRequested()) return;

        Trajectory park = babaji.trajectoryBuilder(depositWhite.end())
                .lineTo(new Vector2d(parkX, parkY))
                .build();

        babaji.followTrajectory(rightTrajPreload);
        babaji.followTrajectory(rightbackdropPreloadTraj);
//        babaji.followTrajectory(midTrajPreload);
//        babaji.followTrajectory(midbackdropPreloadTraj);
//        babaji.followTrajectory(leftTrajPreload);
//        babaji.followTrajectory(leftbackdropPreloadTraj);
        babaji.followTrajectorySequence(intake);
        babaji.followTrajectorySequence(depositWhite);
        babaji.followTrajectorySequence(intake);
        babaji.followTrajectorySequence(depositWhite);
        babaji.followTrajectory(park);
    }
}