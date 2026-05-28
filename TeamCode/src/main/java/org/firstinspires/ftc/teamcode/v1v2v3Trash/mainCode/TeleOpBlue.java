package org.firstinspires.ftc.teamcode.v1v2v3Trash.mainCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.v1v2v3Trash.configurations.parking;
import org.firstinspires.ftc.teamcode.v1v2v3Trash.configurations.shooter;
import org.firstinspires.ftc.teamcode.v1v2v3Trash.configurations.strafe;
import org.firstinspires.ftc.teamcode.v1v2v3Trash.configurations.turelBlue;

@Disabled
public class TeleOpBlue extends OpMode {

    strafe strafe = new strafe();
    parking parking = new parking();
    shooter shooter = new shooter();
    turelBlue turel = new turelBlue();

    @Override
    public void init() {
        strafe.init(hardwareMap);
        parking.init(hardwareMap);
        shooter.init(hardwareMap);
        turel.init(hardwareMap);
    }

    @Override
    public void loop() {
        strafe.baseStrafe(gamepad1.right_trigger, -gamepad1.left_trigger, gamepad1.left_stick_x, gamepad1.right_stick_x);
        parking.tfpark(gamepad1.y);
        shooter.shoot(gamepad1.right_stick_button, gamepad1.a, gamepad1.left_stick_button, gamepad1);
        turel.Turel();
    }
}
