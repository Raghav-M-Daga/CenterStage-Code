package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.Babaji;

@Config
@TeleOp(group = "test")
public class SensorsTest extends LinearOpMode {

    private FtcDashboard dashboard = FtcDashboard.getInstance();
//    public static String pattern = "WHITE";

    @Override
    public void runOpMode() throws InterruptedException {
        Babaji babaji = new Babaji(hardwareMap);
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());

        waitForStart();

        while (!isStopRequested()) {
            babaji.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            babaji.update();

//            if (gamepad1.dpad_up) {
//                babaji.sensors.setLEDPurple();
//            }
//
//            if (gamepad1.dpad_down) {
//                babaji.sensors.setLEDWhite();
//            }
//
//            if (gamepad1.dpad_right || gamepad1.dpad_left) {
//                babaji.sensors.setLED(pattern);
//            }

            telemetry.addData("pixelCount", babaji.sensors.pixelCount());
            telemetry.addData("getFrontDistance", babaji.sensors.getFrontDistance());
//            telemetry.addData("frontDistanceTimedOut", babaji.sensors.frontDistanceTimedOut());
            telemetry.addData("getIntakeFrontDistance", babaji.sensors.getIntakeFrontDistance());
            telemetry.addData("getIntakeBackDistance", babaji.sensors.getIntakeBackDistance());
//            telemetry.addData("ultrasonicDist", babaji.sensors.getUltrasonicDistance());
//            telemetry.addData("Argb", babaji.sensors.getIntakeFrontArgb());
//            telemetry.addData("Red", babaji.sensors.getIntakeFrontRed());
//            telemetry.addData("Green", babaji.sensors.getIntakeFrontGreen());
//            telemetry.addData("Alpha", babaji.sensors.getIntakeFrontAlpha());
            telemetry.update();
        }
    }
}
