package org.firstinspires.ftc.teamcode.mainCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.configTamaraV3.strafe;
import org.firstinspires.ftc.teamcode.configTamaraV3.shooterAndIntake;
@TeleOp
public class blueMain extends OpMode {

    private Servo t1, t2;

    strafe strafe = new strafe();
    shooterAndIntake si = new shooterAndIntake();


    @Override
    public void init() {
        strafe.init(hardwareMap);
        si.init(hardwareMap);
        t1 = hardwareMap.get(Servo.class, "t1");
        t2 = hardwareMap.get(Servo.class, "t2");
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {
        t1.setPosition(0.5);
        t2.setPosition(0.5);
        si.shooterIntake(gamepad1.right_stick_button, gamepad1.left_stick_button, gamepad1.a);
        strafe.baseStrafe(gamepad1.right_trigger, -gamepad1.left_trigger, gamepad1.left_stick_x, gamepad1.right_stick_x);
    }
}
