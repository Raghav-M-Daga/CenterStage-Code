package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.Babaji;

@TeleOp(group = "test")
public class DroneLaunchTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Babaji babaji = new Babaji(hardwareMap);

        waitForStart();

        while (!isStopRequested()) {
            if (gamepad1.dpad_up) {
                babaji.droneLauncher.release();
            }

            if (gamepad1.dpad_down) {
                babaji.droneLauncher.lock();
            }
        }
    }
}
