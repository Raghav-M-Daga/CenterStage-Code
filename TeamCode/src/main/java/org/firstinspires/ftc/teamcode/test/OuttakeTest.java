package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.Babaji;
import org.firstinspires.ftc.teamcode.drive.Outtake;

@TeleOp(group = "test")
public class OuttakeTest extends LinearOpMode {
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

            // Shield + Jonny Prongs Out
            if (gamepad1.right_trigger > 0.2) {
                babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.SHIELD);
                babaji.outtake.setJonnyExtrudePos(Outtake.JonnyExtrudePos.COLLECT);
                while (gamepad1.right_trigger > 0.2) {}
            }
            // Intake Sequence
            if (gamepad1.left_trigger > 0.2) {
                babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.SHIELD);
                babaji.outtake.setJonnyExtrudePos(Outtake.JonnyExtrudePos.COLLECT);
                sleep(1000);
                babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.INTAKE);
                sleep(1000);
                babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.SHIELD);
                while (gamepad1.left_trigger > 0.2) {}
            }
            // Backdrop Outtake Sequence
            if (gamepad1.left_bumper) {
                // This function now transfers in the new Transfer position of the actuator
                babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.BACKDROP);
                sleep(500);
                // Move the actuator to outtake pos after the movement is complete.
//                babaji.outtake.setActuatorRotate(Outtake.ActuatorRotatePos.BACKDROP);
                sleep(1000);
                babaji.outtake.setJonnyExtrudePos(Outtake.JonnyExtrudePos.SCORE);
                while (gamepad1.left_bumper) {}
            }
            // Set to Ground Pose
            if (gamepad1.right_bumper) {
                babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.GROUND);
                sleep(500);
                babaji.outtake.setActuatorRotate(Outtake.ActuatorRotatePos.GROUND);
                sleep(1000);
                babaji.outtake.setJonnyExtrudePos(Outtake.JonnyExtrudePos.SCORE);
//                sleep(2000);
//                babaji.outtake.setJonnyExtrudePos(Outtake.JonnyExtrudePos.COLLECT);
//                while (gamepad1.dpad_down) {}
            }
        }
    }
}
