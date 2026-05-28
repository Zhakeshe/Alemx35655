package org.firstinspires.ftc.teamcode.v1v2v3Trash.TeleOpp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
public class servoa0y05b1 extends OpMode {

    private Servo x2;
    private Servo x3;

    @Override
    public void init() {
        x2 = hardwareMap.get(Servo.class, "x2");
        x3 = hardwareMap.get(Servo.class, "x3");
    }

    @Override
    public void loop() {
        if (gamepad1.a){
            x2.setPosition(0);
            x3.setPosition(0);
        } else if (gamepad1.y) {
            x2.setPosition(0.5);
            x3.setPosition(0.5);
        } else if (gamepad1.b) {
            x2.setPosition(1);
            x3.setPosition(1);
        }

    }


}
