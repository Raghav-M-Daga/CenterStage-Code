package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.Babaji;
import org.firstinspires.ftc.teamcode.drive.Outtake;

@TeleOp(group = "test")
public class JonnyTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Babaji babaji = new Babaji(hardwareMap);

        int jonnyRotatePosesIndex = 0;
        Outtake.JonnyRotatePos jonnyRotatePoses[] = new Outtake.JonnyRotatePos[] {
            Outtake.JonnyRotatePos.INTAKE, Outtake.JonnyRotatePos.VERTICAL, Outtake.JonnyRotatePos.RIGHT, Outtake.JonnyRotatePos.LEFT, Outtake.JonnyRotatePos.GROUND, Outtake.JonnyRotatePos.HORIZONTAL
        };

        waitForStart();

        while (!isStopRequested()) {
            if (gamepad1.right_bumper) {
                babaji.outtake.setJonnyExtrudePos(Outtake.JonnyExtrudePos.COLLECT);
            }
            if (gamepad1.left_bumper) {
                babaji.outtake.setJonnyExtrudePos(Outtake.JonnyExtrudePos.SCORE);
            }
            if (gamepad1.dpad_right) {
                babaji.outtake.setJonnyRotate(Outtake.JonnyRotatePos.VERTICAL);
//                jonnyRotatePosesIndex++;
//                if (jonnyRotatePosesIndex > jonnyRotatePoses.length - 1) {
//                    jonnyRotatePosesIndex = jonnyRotatePoses.length - 1;
//                }
            }
            // left trigger injora using rotate
            // right dpad micro using jonny
            if (gamepad1.dpad_left) {
                babaji.outtake.setJonnyRotate(Outtake.JonnyRotatePos.HORIZONTAL);
//                jonnyRotatePosesIndex--;
//                if (jonnyRotatePosesIndex < 0) {
//                    jonnyRotatePosesIndex = 0;
//                }
            }
        }
    }
}
