package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.strafe;
import org.firstinspires.ftc.teamcode.config.shooter;

@TeleOp
public class mainBlue extends OpMode {

    //intakeCon intakeCon = new intakeCon();
    strafe strafe = new strafe();
    shooter shooter = new shooter();
    //turell turell = new turell();

    @Override
    public void init() {
        //intakeCon.init(hardwareMap);
        strafe.init(hardwareMap);
        shooter.init(hardwareMap);
        //turell.init(hardwareMap);
    }

    public void loop(){
        //intakeCon.intake(gamepad1.right_bumper);
        shooter.shooteryep(gamepad1.a, gamepad1.dpad_up, gamepad1.dpad_down,gamepad1.y, telemetry);
        strafe.baseStrafe(gamepad1.right_trigger, -gamepad1.left_trigger, gamepad1.left_stick_x, gamepad1.right_stick_x);
        //turell.tu();
    }
}
