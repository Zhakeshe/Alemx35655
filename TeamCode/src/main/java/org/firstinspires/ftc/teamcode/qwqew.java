package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.opencv.core.Mat;

@TeleOp
public class qwqew extends OpMode {
    private Servo qwe;

    private DcMotor Motor;
    private DcMotor Motor2;

    @Override
    public void init() {
        qwe = hardwareMap.get(Servo.class, "qwe");
        Motor = hardwareMap.get(DcMotor.class, "Motor");
        Motor2 = hardwareMap.get(DcMotor.class, "Motor2");
        Motor.setDirection(DcMotorSimple.Direction.FORWARD);
        Motor2.setDirection(DcMotor.Direction.REVERSE);
        qwe.setPosition(1);
    }

    @Override
    public void loop() {
        servo();
        motorr();
    }

    private void servo(){
        double y = gamepad1.right_stick_y + 0.5;
        qwe.setPosition(y);
    }

    private void motorr(){
        double r = gamepad1.right_trigger;
        double l = gamepad1.left_trigger;
        double d = Math.max(Math.abs(r) + Math.abs(l), 1);
        Motor.setPower(((r - l) / d) / 2);
        Motor2.setPower(((r - l) / d) / 2);
    }
}
