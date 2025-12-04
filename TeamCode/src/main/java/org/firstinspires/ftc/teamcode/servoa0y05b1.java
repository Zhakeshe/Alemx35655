package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class servoa0y05b1 extends OpMode {

    private Servo o5;

    @Override
    public void init() {
        o5 = hardwareMap.get(Servo.class, "o5");
    }

    @Override
    public void loop() {
        if (gamepad1.a){
            o5.setPosition(0);
        } else if (gamepad1.y) {
            o5.setPosition(0.5);
        } else if (gamepad1.b) {
            o5.setPosition(1);
        }

    }


}
