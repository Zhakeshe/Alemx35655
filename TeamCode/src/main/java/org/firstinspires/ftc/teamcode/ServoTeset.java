package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ServoTeset extends OpMode {
    private Servo xy0;

    @Override
    public void init() {
        xy0 = hardwareMap.get(Servo.class, "xy0");
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            xy0.setPosition(1);
        } else if (gamepad1.b) {
            xy0.setPosition(0);
        }
    }
}
