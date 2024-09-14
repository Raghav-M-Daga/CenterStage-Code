package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.Babaji;

@TeleOp(group = "test")
public class HangTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Babaji babaji = new Babaji(hardwareMap);

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

            if (gamepad1.dpad_up) {
                babaji.hang.raise();
            }

            if (gamepad1.dpad_down) {
                babaji.hang.lower();
            }
        }
    }
}
