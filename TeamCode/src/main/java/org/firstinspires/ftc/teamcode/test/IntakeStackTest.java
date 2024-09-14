package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.Babaji;
import org.firstinspires.ftc.teamcode.drive.Intake;
import org.firstinspires.ftc.teamcode.drive.Outtake;
import org.firstinspires.ftc.teamcode.drive.Sensors;

@TeleOp(group = "test")
public class IntakeStackTest extends LinearOpMode {

    private FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {
        Babaji babaji = new Babaji(hardwareMap);
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());

        waitForStart();

        while (!isStopRequested()) {
            if (gamepad1.dpad_up) {
                babaji.intake.collectPixel5Fixed(babaji);
            } else if (gamepad1.dpad_down) {
                babaji.intake.collectPixelsFarStack(babaji);
            } else if (gamepad1.right_bumper) {
                babaji.intake.raiseIntake();
                while (gamepad1.right_bumper) {}
            } else if (gamepad1.left_bumper) {
                babaji.intake.lowerIntake();
                while (gamepad1.left_bumper) {}
            } else if (gamepad1.a) {
                babaji.intake.collectPixelsFarCycle(babaji);
                while (gamepad1.a) {}
            }
            else {
                babaji.intake.setPow(gamepad1.left_stick_y);
            }

            telemetry.addData("Power", gamepad1.left_stick_y);
            telemetry.addData("Pixel Count", babaji.sensors.pixelCount());
            telemetry.addData("Front Distance", babaji.sensors.getFrontDistance());
            telemetry.update();
        }

        babaji.stop();
    }
}
