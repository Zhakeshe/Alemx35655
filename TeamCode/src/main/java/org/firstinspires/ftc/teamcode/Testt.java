package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Testt", group = "examples")
public class Testt extends LinearOpMode{
    private DcMotor Motor;

    @Override
    public void runOpMode() {
        Motor = hardwareMap.get(DcMotor.class, "Motor");

        waitForStart();
        if (opModeIsActive()){
            while (opModeIsActive()){
                double r = gamepad1.right_trigger;
                double l = gamepad1.left_trigger;
                double d = Math.max(Math.abs(r) + Math.abs(l), 1);
                Motor.setPower((r - l) / d);
            }
        }

    }




}
