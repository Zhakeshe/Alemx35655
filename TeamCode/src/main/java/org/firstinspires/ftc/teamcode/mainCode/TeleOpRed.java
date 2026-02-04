package org.firstinspires.ftc.teamcode.mainCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.configurations.intakeTrans;
import org.firstinspires.ftc.teamcode.configurations.parking;
import org.firstinspires.ftc.teamcode.configurations.shooter;
import org.firstinspires.ftc.teamcode.configurations.strafe;
import org.firstinspires.ftc.teamcode.configurations.turelRed;

@TeleOp
public class TeleOpRed extends OpMode {

    strafe strafe = new strafe();
    parking parking = new parking();
    shooter shooter = new shooter();
    intakeTrans intakeTrans = new intakeTrans();
    turelRed turel = new turelRed();

    @Override
    public void init() {
        strafe.init(hardwareMap);
        parking.init(hardwareMap);
        shooter.init(hardwareMap);
        intakeTrans.init(hardwareMap);
        turel.init(hardwareMap);
    }

    @Override
    public void loop() {
        strafe.baseStrafe(gamepad1.right_trigger, -gamepad1.left_trigger, gamepad1.left_stick_x, gamepad1.right_stick_x);
        parking.tfpark(gamepad1.y);
        shooter.shoot(gamepad1.left_stick_button, gamepad1);
        intakeTrans.Intake(gamepad1.right_stick_button, gamepad1.a, gamepad1);
        turel.Turel();
    }
}
