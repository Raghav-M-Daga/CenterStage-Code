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
public class ThreadTest extends LinearOpMode {
    public static double slideEncCount = 10000;
    Babaji babaji;


    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        babaji = new Babaji(hardwareMap);
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());

        double increment = 0;
        babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.SHIELD);
        Thread extendOuttake1 = new ExtendOuttake(1);
        Thread extendOuttake2 = new ExtendOuttake(2);
        Thread transfer = new Transfer();

        waitForStart();

        while (!isStopRequested()) {
            if (gamepad1.dpad_up && !extendOuttake1.isAlive()) {
                extendOuttake1.start();
            } else if (gamepad1.dpad_down && !extendOuttake2.isAlive()) {
                extendOuttake2.start();
            } else if (gamepad1.dpad_right && !transfer.isAlive()) {
                transfer.start();
            }

            telemetry.update();
        }
    }
    private class ExtendOuttake extends Thread {
    private int version;

    public ExtendOuttake(int version) {
        this.version = version;
    }
    @Override
    public void run()
    {
        try
        {
            if (version == 1) {
                babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.BACKDROP);
                sleep(400);
                babaji.outtake.setJonnyRotate(Outtake.JonnyRotatePos.HORIZONTAL);
            } else if (version == 2) {
                babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.BACKDROP);
                while (babaji.outtake.getEncCount() < slideEncCount) {
                    babaji.outtake.setSlidesPow(Outtake.slidesPosPow);
                }
                babaji.outtake.setSlidesPow(Outtake.restPow);
            }
        }
        catch (Exception e) {}
    }
}

    private class Transfer extends Thread {
    @Override
    public void run()
    {
        try
        {
            babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.PREP);
            babaji.outtake.setJonnyExtrudePos(Outtake.JonnyExtrudePos.COLLECT);
            sleep(500);
            babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.INTAKE);
            sleep(300);
            babaji.intake.setPow(0);
            babaji.outtake.setOuttakeRotate(Outtake.OuttakeRotatePos.SHIELD);
        }
        catch (Exception e) {}
    }
} }