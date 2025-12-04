package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Testt extends LinearOpMode{
    private DcMotor Motor;
    private DcMotor Motor2;
    private Servo qwe;

    @Override
    public void runOpMode() {
        Motor = hardwareMap.get(DcMotor.class, "Motor");
        Motor2 = hardwareMap.get(DcMotor.class, "Motor2");
        qwe = hardwareMap.get(Servo.class, "qwe");

        Motor.setDirection(DcMotorSimple.Direction.FORWARD);
        Motor2.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        if (opModeIsActive()){
            while (opModeIsActive()){
                double r = gamepad1.right_trigger;
                double l = gamepad1.left_trigger;
                double d = Math.max(Math.abs(r) + Math.abs(l), 1);
                Motor.setPower((r - l) / d);
                Motor2.setPower((r - l) / d);

                double y = gamepad1.right_stick_y + 0.5;
                qwe.setPosition(y);
            }
        }

    }




}
