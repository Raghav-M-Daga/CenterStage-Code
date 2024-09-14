package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

public class EnhancedGamepadTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Gamepad.LedEffect rgbEffect = new Gamepad.LedEffect.Builder()
                .addStep(1, 0, 0, 250) // Show red for 250ms
                .addStep(0, 1, 0, 250) // Show green for 250ms
                .addStep(0, 0, 1, 250) // Show blue for 250ms
                .addStep(1, 1, 1, 250) // Show white for 250ms
                .build();

        waitForStart();

        while (!isStopRequested()) {
            if (gamepad1.a) {
                gamepad1.setLedColor(1,0,0,500);
            }
            if (gamepad1.b) {
                gamepad1.setLedColor(0,1,0,500);
            }
            if (gamepad1.x) {
                gamepad1.setLedColor(0,0,1,500);
            }
            if (gamepad1.dpad_up) {
                gamepad1.setLedColor(1,1,0,500);
            }
            if (gamepad1.dpad_down) {
                gamepad1.setLedColor(1,0,1,500);
            }
            if (gamepad1.dpad_left) {
                gamepad1.setLedColor(0,1,1,500);
            }
            if (gamepad1.dpad_right) {
                gamepad1.setLedColor(1,1,1,500);
            }
            if (gamepad1.right_bumper) {
                gamepad1.rumble(500);
            }
            if (gamepad1.left_bumper) {
                gamepad1.stopRumble();
            }

            telemetry.addData("touchpad x", gamepad1.touchpad_finger_1_x);
            telemetry.addData("touchpad y", gamepad1.touchpad_finger_1_y);
            telemetry.update();
        }
    }
}
