package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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
public class AutoRedFarCycle extends LinearOpMode {
    OpenCvCamera camera;
    DetectionPipeline detectionPipeline;
    Sensors sensor;
    Babaji babaji = new Babaji(hardwareMap);
    public static double preloadX = -27, preloadY = 0.5, preloadAngle = 0;
    public static double intakeX = -26, intakeX2 = -24.5, intakeY = -21, intakeY2 = -20, intakeYStep = -5, intakeAngle = 269;
    public static double setUpX = -27;
    public static double straightEndX = -25.5, straightEndY = 60, straightInterpol = 90;
    public static double backdropScoreAngle = 270, backdropScoreInterpol = 90, backDropScoreY = 86.5;
    public static double backdropScoreWhites = -37, backDropScoreY2 = 86;
    public static double backdropScoreYellow = -33, backdropScoreWhite = -17;
    public static double parkX = -5, parkY = 77;
    public static double slideEncCount = 8000;
    public static double maxVA = 70;
    public static double lowVA = 30;
    public static double outtakeToGroundDist = 0.07, outtakeToGroundOffset = 0, preloadToBackdropDist = 0.8, preloadToBackdropOffset = 0.0, backdropToIntakeDist = 0.2, backdropToIntakeOffset = 0;
    public static double strafeDist = 5.5;
//    public static int pos = 3;

    @Override
    public void runOpMode() throws InterruptedException {
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

        // Middle Line Trajectory

        TrajectorySequence midTrajPreload = babaji.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(preloadX, preloadY, Math.toRadians(preloadAngle)))
                .build();

        TrajectorySequence midTrajIntake = babaji.trajectorySequenceBuilder(midTrajPreload.end())
                .setReversed(false)
                .setVelConstraint((v, pose2d, pose2d1, pose2d2) -> lowVA)
                .setAccelConstraint((a, pose2d, pose2d1, pose2d2) -> lowVA)
                .lineToLinearHeading(new Pose2d(intakeX, intakeY, Math.toRadians(intakeAngle)))
//                .addDisplacementMarker(() -> {
//                    babaji.outtake.setJonnyExtrudePos(Outtake.JonnyExtrudePos.SCORE);
//                })
                .build();

        if (isStopRequested()) return;

        babaji.followTrajectorySequence(midTrajPreload);
//        sleep(500);
        // just push to drop the pixel
        babaji.followTrajectorySequence(midTrajIntake);
        babaji.outtake.setJonnyExtrudePos(Outtake.JonnyExtrudePos.SCORE);
//        sleep(200);
        babaji.intake.collectPixel5Fixed(babaji);
        babaji.intake.raiseIntakeMoving();
        babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.PREP);
        babaji.outtake.setJonnyExtrudePos(Outtake.JonnyExtrudePos.COLLECT);
        sleep(300);
        babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.INTAKE);
        sleep(200);
        babaji.intake.setPow(0);
        TrajectorySequence midTrajBackdrop = babaji.trajectorySequenceBuilder(babaji.getPoseEstimate())
                .setReversed(true)
                .setVelConstraint((v, pose2d, pose2d1, pose2d2) -> maxVA)
                .setAccelConstraint((a, pose2d, pose2d1, pose2d2) -> maxVA)
                .splineToConstantHeading(new Vector2d(straightEndX, straightEndY), Math.toRadians(straightInterpol))
                .addDisplacementMarker(preloadToBackdropDist, preloadToBackdropOffset, () -> {
                    babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.BACKDROP);
                })
                .setVelConstraint((v, pose2d, pose2d1, pose2d2) -> lowVA)
                .setAccelConstraint((a, pose2d, pose2d1, pose2d2) -> lowVA)
                .splineToConstantHeading(new Vector2d(backdropScoreYellow, backDropScoreY), Math.toRadians(backdropScoreInterpol))
                .build();
        babaji.followTrajectorySequence(midTrajBackdrop);
        babaji.outtake.setJonnyRotate(Outtake.JonnyRotatePos.HORIZONTAL);
        sleep(600);
        babaji.outtake.setJonnyExtrudePos(Outtake.JonnyExtrudePos.SCORE);
        sleep(400);
        babaji.outtake.setJonnyRotate(Outtake.JonnyRotatePos.VERTICAL);
        babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.SHIELD);
        TrajectorySequence intakeCycle = babaji.trajectorySequenceBuilder(midTrajBackdrop.end())
                .setReversed(false)
                .setVelConstraint((v, pose2d, pose2d1, pose2d2) -> maxVA)
                .setAccelConstraint((a, pose2d, pose2d1, pose2d2) -> maxVA)
                .splineToConstantHeading(new Vector2d(setUpX, straightEndY), Math.toRadians(intakeAngle))
                .splineToConstantHeading(new Vector2d(intakeX, intakeYStep), Math.toRadians(intakeAngle))
                .setVelConstraint((v, pose2d, pose2d1, pose2d2) -> lowVA)
                .setAccelConstraint((a, pose2d, pose2d1, pose2d2) -> lowVA)
                .splineToConstantHeading(new Vector2d(intakeX2, intakeY), Math.toRadians(intakeAngle))
                .build();
        babaji.followTrajectorySequence(intakeCycle);
        babaji.intake.collectPixelsFarCycle(babaji);
        babaji.intake.raiseIntakeMoving();
        babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.PREP);
        babaji.outtake.setJonnyExtrudePos(Outtake.JonnyExtrudePos.COLLECT);
        sleep(500);
        babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.INTAKE);
        sleep(200);
        babaji.intake.setPow(0);
        TrajectorySequence midTrajBackdrop2 = babaji.trajectorySequenceBuilder(babaji.getPoseEstimate())
                .setReversed(true)
                .setVelConstraint((v, pose2d, pose2d1, pose2d2) -> maxVA)
                .setAccelConstraint((a, pose2d, pose2d1, pose2d2) -> maxVA)
                .splineToConstantHeading(new Vector2d(straightEndX, straightEndY), Math.toRadians(straightInterpol))
                .addDisplacementMarker(preloadToBackdropDist, preloadToBackdropOffset, () -> {
                    babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.BACKDROP);
                })
                .setVelConstraint((v, pose2d, pose2d1, pose2d2) -> lowVA)
                .setAccelConstraint((a, pose2d, pose2d1, pose2d2) -> lowVA)
                .splineToConstantHeading(new Vector2d(backdropScoreWhites, backDropScoreY2), Math.toRadians(backdropScoreInterpol))
                .build();
        babaji.followTrajectorySequence(midTrajBackdrop2);
        while (babaji.outtake.getEncCount() < slideEncCount){
            babaji.outtake.setSlidesPow(Outtake.slidesPosPow);
        }
        babaji.outtake.setSlidesPow(Outtake.restPow);
        sleep(400);
        babaji.outtake.setJonnyExtrudePos(Outtake.JonnyExtrudePos.SCORE);
        sleep(400);
        babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.SHIELD);
        sleep(600);
    }

//    private class ExtendOuttake extends Thread {
//        @Override
//        public void run()
//        {
//            try
//            {
//                babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.BACKDROP);
//                while (babaji.outtake.getEncCount() < slideEncCount) {
//                    babaji.outtake.setSlidesPow(Outtake.slidesPosPow);
//                }
//                babaji.outtake.setJonnyRotate(Outtake.JonnyRotatePos.HORIZONTAL);
//            }
//            catch (Exception e) {}
//        }
//    }
}