package org.firstinspires.ftc.teamcode.v1v2v3Trash.configurations;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class parking{
    private Servo p1, p2;

    boolean tfpark1 = false;
    boolean tfpark2 = true;

    public void init(HardwareMap hwMap) {
        p1 = hwMap.get(Servo.class, "p1");
        p2 = hwMap.get(Servo.class, "p2");
        p1.setPosition(0.5);
        p2.setPosition(0.5);
    }

    public void tfpark(boolean g1y) {
        if (g1y & !tfpark1){
            tfpark2 = false;
            if(p1.getPosition() == 0.15){
                tfpark1 = true;
                tfpark2 = true;
                p1.setPosition(0.5);
                p2.setPosition(0.5);
            } else if (p1.getPosition() == 0.5) {
                tfpark1 = true;
                tfpark2 = true;
                p1.setPosition(0.15);
                p2.setPosition(0.85);
            }
        } else if (!g1y && tfpark2) {
            tfpark1 = false;
        }
    }
}
