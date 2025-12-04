package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
public class ServoTest0 extends OpMode {
    private Servo qwe;
    private Servo qwe2;

    private DcMotor motor;
    private DcMotor motor2;

    @Override
    public void init() {
        qwe = hardwareMap.get(Servo.class, "qwe");
        qwe2 = hardwareMap.get(Servo.class, "qwe2");

        motor = hardwareMap.get(DcMotor.class, "motor");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");

        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor2.setDirection(DcMotorSimple.Direction.FORWARD);

    }

    @Override
    public void loop() {
        double x = gamepad1.right_stick_x;
        qwe.setPosition(x + 0.5);
        qwe2.setPosition(x + 0.5);

        double r = gamepad1.right_trigger;
        double l = gamepad1.left_trigger;
        telemetry.addData("r", r);
        telemetry.addData("l", l); 
        motor.setPower((r - l) * 3);
        motor2.setPower((r - l) * 3);
    }
}
