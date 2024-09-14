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
public class AutoRedFarGate extends LinearOpMode {
    OpenCvCamera camera;
    DetectionPipeline detectionPipeline;
    Sensors sensor;
    Babaji babaji;
    public static double preloadX = -27, preloadY = 0.5, preloadAngle = 0;
    public static double intakeX = -25.5, intakeX2 = -47, intakeX3 = -49, intakeY = -18, intakeY2 = -16, intakeYStep = 5, intakeAngle = 268, intakeAngle2 = 275;
    public static double backdropStepX = -23;
    public static double setUpX = -54, setUpY = 50;
    public static double straightEndX = -50, straightEndY = 50, straightInterpol = 90;
    public static double backdropScoreAngle = 270, backdropScoreInterpol = 90, backDropScoreY = 88;
    public static double backdropScoreWhites = -31, backDropScoreY2 = 87.5;
    public static double backdropScoreYellow = -31  , backdropScoreWhite = -17;
    public static double parkX = -5, parkY = 77;
    public static double slideEncCount = 4500;
    public static double slideEncCount2 = 8000;
    public static double maxVA = 65;
    public static double lowVA = 40;
    public static double backdropToIntakeDist = 0.1, backdropToIntakeOffset = 0, preloadToBackdropDist = 0.7, preloadToBackdropOffset = 0.0, transferInBackdropDist = 0.05, transferInBackdropOffset = 0;
    public static double strafeDist = 5.5;
    public static long outtakeTime1 = 500, outtakeTime2 = 750, outtakeTime3 = 600;
//    public static int pos = 3;

    @Override
    public void runOpMode() throws InterruptedException {
        babaji = new Babaji(hardwareMap);
        sensor = new Sensors(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "backCamera"), cameraMonitorViewId);
        detectionPipeline = new DetectionPipeline();

        camera.setPipeline(detectionPipeline);
        Thread extendOuttake1 = new ExtendOuttake(1);
        Thread extendOuttake2 = new ExtendOuttake(2);
        Thread extendOuttake3 = new ExtendOuttake(3);
        Thread transfer = new Transfer();
        Thread retract = new Retract();
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
//                .setVelConstraint((v, pose2d, pose2d1, pose2d2) -> lowVA)
//                .setAccelConstraint((a, pose2d, pose2d1, pose2d2) -> lowVA)
                .lineToLinearHeading(new Pose2d(intakeX, intakeY, Math.toRadians(intakeAngle)))
                .build();
        
        if (isStopRequested()) return;

        babaji.followTrajectorySequence(midTrajPreload);
//        sleep(500);
        // just push to drop the pixel
        babaji.followTrajectorySequence(midTrajIntake);
//        babaji.outtake.setJonnyExtrudePos(Outtake.JonnyExtrudePos.SCORE);
      //  sleep(200);
        babaji.intake.collectPixel5Fixed(babaji);
        babaji.intake.raiseIntakeMoving();
        TrajectorySequence midTrajBackdrop = babaji.trajectorySequenceBuilder(babaji.getPoseEstimate())
                .setReversed(true)
                .setVelConstraint((v, pose2d, pose2d1, pose2d2) -> maxVA)
                .setAccelConstraint((a, pose2d, pose2d1, pose2d2) -> maxVA)
                .addDisplacementMarker(transferInBackdropDist, transferInBackdropOffset, () -> {
                    transfer.start();
                })
                .splineToSplineHeading(new Pose2d(backdropStepX, straightEndY, Math.toRadians(backdropScoreAngle)), Math.toRadians(straightInterpol))
                .addDisplacementMarker(preloadToBackdropDist, preloadToBackdropOffset, () -> {
                    extendOuttake1.start();
                })
                .setVelConstraint((v, pose2d, pose2d1, pose2d2) -> lowVA)
                .setAccelConstraint((a, pose2d, pose2d1, pose2d2) -> lowVA)
                .splineToConstantHeading(new Vector2d(backdropScoreYellow, backDropScoreY), Math.toRadians(backdropScoreInterpol))
                .build();
//        sleep(200);
        babaji.followTrajectorySequence(midTrajBackdrop);
//        babaji.outtake.setJonnyExtrudePos(Outtake.JonnyExtrudePos.SCORE);
        sleep(200);
        TrajectorySequence intakeCycle = babaji.trajectorySequenceBuilder(midTrajBackdrop.end())
                .setReversed(false)
                .setVelConstraint((v, pose2d, pose2d1, pose2d2) -> maxVA)
                .setAccelConstraint((a, pose2d, pose2d1, pose2d2) -> maxVA)
                .addDisplacementMarker(backdropToIntakeDist, backdropToIntakeOffset, () -> {
                    babaji.outtake.setJonnyRotate(Outtake.JonnyRotatePos.VERTICAL);
                    babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.SHIELD);
                })
                .splineToConstantHeading(new Vector2d(setUpX, setUpY), Math.toRadians(intakeAngle2))
                .splineToSplineHeading(new Pose2d(intakeX2, intakeYStep, Math.toRadians(intakeAngle)), Math.toRadians(intakeAngle))
                .setVelConstraint((v, pose2d, pose2d1, pose2d2) -> lowVA)
                .setAccelConstraint((a, pose2d, pose2d1, pose2d2) -> lowVA)
                .splineToConstantHeading(new Vector2d(intakeX2, intakeY2), Math.toRadians(intakeAngle))
                .build();
        babaji.followTrajectorySequence(intakeCycle);
        babaji.intake.collectPixelsFarStack(babaji);
        babaji.intake.raiseIntakeMoving();
        TrajectorySequence midTrajBackdrop2 = babaji.trajectorySequenceBuilder(babaji.getPoseEstimate())
                .setReversed(true)
                .setVelConstraint((v, pose2d, pose2d1, pose2d2) -> maxVA)
                .setAccelConstraint((a, pose2d, pose2d1, pose2d2) -> maxVA)
                .addDisplacementMarker(transferInBackdropDist, transferInBackdropOffset, () -> {
                    transfer.start();
                })
                .splineToSplineHeading(new Pose2d(straightEndX, straightEndY, Math.toRadians(backdropScoreAngle)), Math.toRadians(straightInterpol))
                .addDisplacementMarker(preloadToBackdropDist, preloadToBackdropOffset, () -> {
                    extendOuttake2.start();
                })
                .setVelConstraint((v, pose2d, pose2d1, pose2d2) -> lowVA)
                .setAccelConstraint((a, pose2d, pose2d1, pose2d2) -> lowVA)
                .splineToConstantHeading(new Vector2d(backdropScoreWhites, backDropScoreY2), Math.toRadians(backdropScoreInterpol))
                .build();
        babaji.followTrajectorySequence(midTrajBackdrop2);
        sleep(200);
        retract.start();
//        babaji.outtake.setJonnyExtrudePos(Outtake.JonnyExtrudePos.SCORE);
//        sleep(400);

        //COMMENTED OUT THE LAST 2 PIXELS IN A 2+5
//        TrajectorySequence intakeCycle2 = babaji.trajectorySequenceBuilder(midTrajBackdrop2.end())
//                .setReversed(false)
//                .setVelConstraint((v, pose2d, pose2d1, pose2d2) -> maxVA)
//                .setAccelConstraint((a, pose2d, pose2d1, pose2d2) -> maxVA)
////                .addDisplacementMarker(backdropToIntakeDist, backdropToIntakeOffset, () -> {
////                    retract.start();
////                })
//                .splineToConstantHeading(new Vector2d(setUpX, straightEndY), Math.toRadians(intakeAngle))
//                //try this later but experiment: see if it will fix the weird coming in at an angle by making it a constant heading
////                .splineToConstantHeading(new Vector2d(setUpX, intakeYStep), Math.toRadians(intakeAngle))
//                .splineToSplineHeading(new Pose2d(setUpX, intakeYStep, Math.toRadians(intakeAngle)), Math.toRadians(intakeAngle))
//                .setVelConstraint((v, pose2d, pose2d1, pose2d2) -> lowVA)
//                .setAccelConstraint((a, pose2d, pose2d1, pose2d2) -> lowVA)
//                .splineToConstantHeading(new Vector2d(intakeX3, intakeY2), Math.toRadians(intakeAngle))
//                .build();
//        babaji.followTrajectorySequence(intakeCycle2);
////        babaji.intake.collectLowerPixelsFar(babaji);
////        babaji.intake.raiseIntakeMoving();
//        TrajectorySequence midTrajBackdrop3 = babaji.trajectorySequenceBuilder(babaji.getPoseEstimate())
//                .setReversed(true)
//                .setVelConstraint((v, pose2d, pose2d1, pose2d2) -> maxVA)
//                .setAccelConstraint((a, pose2d, pose2d1, pose2d2) -> maxVA)
////                .addDisplacementMarker(transferInBackdropDist, transferInBackdropOffset, () -> {
////                    transfer.start();
////                })
//                .splineToSplineHeading((new Pose2d(straightEndX, straightEndY, Math.toRadians(backdropScoreAngle))), Math.toRadians(straightInterpol))
////                .addDisplacementMarker(preloadToBackdropDist, preloadToBackdropOffset, () -> {
////                    extendOuttake3.start();
////                })
//                .setVelConstraint((v, pose2d, pose2d1, pose2d2) -> lowVA)
//                .setAccelConstraint((a, pose2d, pose2d1, pose2d2) -> lowVA)
//                .splineToConstantHeading(new Vector2d(backdropScoreWhites, backDropScoreY2), Math.toRadians(backdropScoreInterpol))
//                .build();
//        babaji.followTrajectorySequence(midTrajBackdrop3);
////        babaji.outtake.setJonnyExtrudePos(Outtake.JonnyExtrudePos.SCORE);
////        sleep(500);
//        retract.start();
//        babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.SHIELD);
//        while (Math.abs(babaji.outtake.getEncCount() - Outtake.ground) > Outtake.allowableError) {
//            babaji.outtake.setSlidesPow(Outtake.slidesNegPow);
//        }
//        babaji.outtake.setSlidesPow(Outtake.restPow);
        sleep(200);
    }

    private class ExtendOuttake extends Thread {
        private int version;

        public ExtendOuttake(int version) {
            this.version = version;
        }
        @Override
        public void run()
        {
            try {
                if (version == 1) {
                    babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.BACKDROP);
                    sleep(500);
                    babaji.outtake.setJonnyRotate(Outtake.JonnyRotatePos.HORIZONTAL);
                    sleep(outtakeTime1);
                    babaji.outtake.setJonnyExtrudePos(Outtake.JonnyExtrudePos.SCORE);
                } else if (version == 2){
                    babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.BACKDROP);
                    while (babaji.outtake.getEncCount() < slideEncCount) {
                        babaji.outtake.setSlidesPow(Outtake.slidesPosPow);
                    }
                    babaji.outtake.setSlidesPow(Outtake.restPow);
                    sleep(outtakeTime2);
                    babaji.outtake.setJonnyExtrudePos(Outtake.JonnyExtrudePos.SCORE);
                } else if (version == 3) {
                    babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.BACKDROP);
                    while (babaji.outtake.getEncCount() < slideEncCount2) {
                        babaji.outtake.setSlidesPow(Outtake.slidesPosPow);
                    }
                    babaji.outtake.setSlidesPow(Outtake.restPow);
                    sleep(outtakeTime3);
                    babaji.outtake.setJonnyExtrudePos(Outtake.JonnyExtrudePos.SCORE);
                }
            }
            catch (Exception e) {}
        }
    }

    private class Transfer extends Thread {
        @Override
        public void run()
        {
            try
            {
                babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.PREP);
                babaji.outtake.setJonnyExtrudePos(Outtake.JonnyExtrudePos.COLLECT);
                sleep(500);
                babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.INTAKE);
                sleep(200);
                babaji.intake.setPow(0);
            }
            catch (Exception e) {}
        }
    }
    private class Retract extends Thread {
        @Override
        public void run()
        {
            try
            {
                babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.SHIELD);
                while (Math.abs(babaji.outtake.getEncCount() - Outtake.ground) > Outtake.allowableError) {
                    babaji.outtake.setSlidesPow(Outtake.slidesNegPow);
                }
                babaji.outtake.setSlidesPow(Outtake.restPow);
            }
            catch (Exception e) {}
        }
    }
}