package org.firstinspires.ftc.teamcode.mainCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class parking extends OpMode {

    private Servo p0;
    private Servo p1;

    @Override
    public void init() {
        p0 = hardwareMap.get(Servo.class, "p0");
        p1 = hardwareMap.get(Servo.class, "p1");
    }

    @Override
    public void loop() {
        if (gamepad1.left_stick_button){
            p0.setPosition(0);
            p1.setPosition(1);
        } else if (gamepad1.right_stick_button) {
            p0.setPosition(0.5);
            p1.setPosition(0.5);
        } 
    }
}
