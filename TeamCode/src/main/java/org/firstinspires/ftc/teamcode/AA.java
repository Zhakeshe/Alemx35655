package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
public class AA extends OpMode {
    private Servo o4;

    @Override
    public void init() {
        o4 = hardwareMap.get(Servo.class, "o4");

    }

    @Override
    public void loop() {
        if (gamepad1.a){
            o4.setPosition(90);
        } else if (gamepad1.b) {
            o4.setPosition(85);
        } else if (gamepad1.y) {
            o4.setPosition(65);
        }
    }
}
