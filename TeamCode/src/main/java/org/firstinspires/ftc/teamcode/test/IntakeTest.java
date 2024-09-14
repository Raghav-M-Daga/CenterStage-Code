package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.Babaji;
import org.firstinspires.ftc.teamcode.drive.Intake;
import org.firstinspires.ftc.teamcode.drive.Outtake;

@TeleOp(group = "test")
public class IntakeTest extends LinearOpMode {

    private FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {
        Babaji babaji = new Babaji(hardwareMap);
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());

        double increment = 0;
        babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.SHIELD);

        waitForStart();

        while (!isStopRequested()) {
            if (gamepad1.a) {
                babaji.outtake.setJonnyExtrudePos(Outtake.JonnyExtrudePos.SCORE);
                babaji.intake.collectPixels();
            } else if (gamepad1.right_bumper) {
                babaji.intake.raiseIntake();
            } else if (gamepad1.left_bumper) {
                babaji.intake.lowerIntake();
            } else if (gamepad1.dpad_up) {
                babaji.intake.intakePixel5();
//                increment += 0.01;
//                babaji.intake.incrementRotate(increment);
                while (gamepad1.dpad_up) {}
            } else if (gamepad1.dpad_down) {
                babaji.intake.intakePixel4();
//                increment -= 0.01;
//                babaji.intake.incrementRotate(increment);
                while (gamepad1.dpad_down) {}
            } else if (gamepad1.dpad_right) {
                babaji.intake.intakePixel3();
//                increment += 0.01;
//                babaji.intake.incrementRotate(increment);
                while (gamepad1.dpad_right) {
                }
            } else if (gamepad1.dpad_left) {
                babaji.intake.intakePixel2();
//                increment += 0.01;
//                babaji.intake.incrementRotate(increment);
                while (gamepad1.dpad_left) {}
            } else if (gamepad1.b) {
                babaji.intake.intakePixel1();
            } else {
                babaji.intake.setPow(gamepad1.left_stick_y);
            }

            if (gamepad1.right_trigger > 0.2) {
                babaji.intake.intakePixel5();
                babaji.intake.setPow(Intake.collectPow);
                sleep(3000);
//                babaji.intake.intakePixel4();
//                babaji.intake.setPow(Intake.collectPow);
//                sleep(1000);
                babaji.intake.setPow(0.0);
            }

            telemetry.addData("Power", gamepad1.left_stick_y);
            telemetry.addData("Pixel Count", babaji.sensors.pixelCount());
            telemetry.update();
        }
    }
}
