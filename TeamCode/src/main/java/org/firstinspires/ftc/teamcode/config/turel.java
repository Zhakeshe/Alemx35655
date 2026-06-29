package org.firstinspires.ftc.teamcode.config;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class turel extends OpMode {

    Servo t1, t2;

    DcMotorEx encoder;

    @Override
    public void init() {
        encoder = hardwareMap.get(DcMotorEx.class, "shooter2");
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        t1 = hardwareMap.get(Servo.class ,"t1");
        t2 = hardwareMap.get(Servo.class ,"t2");

    }

    @Override
    public void loop() {
        telemetry.addData("", encoder.getCurrentPosition());
        double last = 0 ;
        if (gamepad1.a){
            t1.setPosition(0);
            t2.setPosition(0);
        }else if(gamepad1.b){
            t1.setPosition(1);
            t2.setPosition(1);
        } else if (gamepad1.y) {
            t1.setPosition(0.5);
            t2.setPosition(0.5);
        }
    }
}
