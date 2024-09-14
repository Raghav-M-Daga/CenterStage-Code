//package org.firstinspires.ftc.teamcode.endgame;
//
//import android.util.Size;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
//import org.firstinspires.ftc.teamcode.drive.Babaji;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
//import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
//
//
//@Config
//@TeleOp
//public class AprilTagDetection extends LinearOpMode {
//    public static int tagID = 5;
//    public static int targetDist = 15;
//    Babaji babaji = new Babaji(hardwareMap);
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        //Set up april tag processor and .set values are projections for camera on tags
//
//        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
//                .setDrawAxes(true)
//                .setDrawCubeProjection(true)
//                .setDrawTagID(true)
//                .setDrawTagOutline(true)
//                .build();
//
//        //Swt up vision portal to get april tag info from camera
//
//        VisionPortal visionPortal = new VisionPortal.Builder()
//                .addProcessor(tagProcessor)
//                .setCamera(hardwareMap.get(WebcamName.class, "backWebcam"))
//                .setCameraResolution(new Size(640, 488))
//                .build();
//        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {}
//
//        GainControl gain = visionPortal.getCameraControl(GainControl.class);
//        gain.setGain(255); // change value
//
//        double desired_x = 0, desired_y = 0, desired_heading = 0;
//        boolean goToTag = false;
//
//        ElapsedTime timer = new ElapsedTime();
//
//        waitForStart();
//
//        while (!isStopRequested() && opModeIsActive()) {
//
//// Confirm if more thn 0 april tags are seen
//            if (tagProcessor.getDetections().size() > 0) {
//                org.firstinspires.ftc.vision.apriltag.AprilTagDetection tag = tagProcessor.getDetections().get(0);
//
//               //idk what is tag exactly or the error w the get detections
//                for (tag : tagProcessor.getDetections().get(0)) {
//                    if (tag.id == tagID) {
//                        desired_y = tag.ftcPose.x;
//                        desired_x = -1 * tag.ftcPose.y + targetDist;
//                        desired_heading = angleWrap(-1 * tag.ftcPose.yaw);
//
//                        telemetry.addData("x", tag.ftcPose.x);
//                        telemetry.addData("y", tag.ftcPose.y);
////                        telemetry.addData("z", tag.ftcPose.z);
////                        telemetry.addData("roll", tag.ftcPose.roll);
////                        telemetry.addData("pitch", tag.ftcPose.pitch);
//                        telemetry.addData("yaw", tag.ftcPose.yaw);
//                        telemetry.update();
//                    }
//                }
//
//
//                // x, y , and range poses are the only relevant to get to april tag.
//                telemetry.addData("x", tag.ftcPose.x);
//                telemetry.addData("y", tag.ftcPose.y);
//                telemetry.addData("range", tag.ftcPose.range);
//
//                telemetry.update();
//
//            }
//
//            TrajectorySequence toTag = babaji.trajectorySequenceBuilder(babaji.getPoseEstimate())
//                    .lineToLinearHeading(new Pose2d(desired_x, desired_y, Math.toRadians(desired_heading)))
//                    .build();
//
//            if (goToTag){
//                babaji.followTrajectorySequence(toTag);
//            }
//        }
//    }
//    public double angleWrap(double radians) {
//
//        while (radians > Math.PI) {
//            radians -= 2 * Math.PI;
//        }
//        while (radians < -Math.PI) {
//            radians += 2 * Math.PI;
//        }
//
//        // keep in mind that the result is in radians
//        return radians;
//    }
//}
//
//
