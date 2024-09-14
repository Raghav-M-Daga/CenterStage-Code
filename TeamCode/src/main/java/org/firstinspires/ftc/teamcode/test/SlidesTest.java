package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.Babaji;
import org.firstinspires.ftc.teamcode.drive.Outtake;

@Config
@TeleOp(group = "test")
public class SlidesTest extends LinearOpMode {

    public static double targetPos = 100;
    public static int level = 1;
    public Babaji babaji;

    @Override
    public void runOpMode() throws InterruptedException {
        babaji = new Babaji(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Raise raise = new Raise();

        waitForStart();

        while (!isStopRequested()) {
            babaji.outtake.setSlidesPow(gamepad1.left_stick_y);

            if (gamepad1.a) {
//                babaji.outtake.raiseToHeight(targetPos);
            }
            if (gamepad1.y) {
                switch (level) {
                    case 0:
                        babaji.outtake.raiseToHeight(Outtake.DepositLevel.GROUND);
                        break;
                    case 1:
                        babaji.outtake.raiseToHeight(Outtake.DepositLevel.LOW);
                        break;
                    case 2:
                        babaji.outtake.raiseToHeight(Outtake.DepositLevel.MID);
                        break;
                    case 3:
                        babaji.outtake.raiseToHeight(Outtake.DepositLevel.HIGH);
                        break;
                }
            }

            telemetry.addData("Current Count", babaji.outtake.getEncCount());
            telemetry.addData("Current", babaji.outtake.getSlidesCurrent());
//            telemetry.addData("Error", targetPos - babaji.outtake.getEncCount());
            telemetry.addData("Pow", gamepad1.left_stick_y);
            telemetry.update();
        }

        raise.interrupt();
    }

    private class Raise extends Thread {
        public Raise() {}

        @Override
        public void run() {
            try {
//                babaji.outtake.raiseToHeight(targetPos);
            } catch (Exception e) {}

            this.interrupt();
        }
    }
}
