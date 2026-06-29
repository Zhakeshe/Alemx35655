package org.firstinspires.ftc.teamcode.Test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@Config
@TeleOp(name = "TurretTest")
public class TurretTest extends LinearOpMode {

    private Servo turretr;
    private Servo turretl;
    public static double turret90deegreepos = 0.8;

    @Override
    public void runOpMode() {
        turretr = hardwareMap.get(Servo.class, "turel");
        turretl = hardwareMap.get(Servo.class, "turel1");

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                turretr.setPosition(0.5);
                turretl.setPosition(0.5);
            } else if (gamepad1.x) {
                turretr.setPosition(0.0);
                turretl.setPosition(0.0);
            } else if (gamepad1.b) {
                turretr.setPosition(turret90deegreepos);
                turretl.setPosition(turret90deegreepos);
            }

            telemetry.addData("turretr Pos", turretr.getPosition());
            telemetry.addData("turretl Pos", turretl.getPosition());
            telemetry.update();
        }
    }
}