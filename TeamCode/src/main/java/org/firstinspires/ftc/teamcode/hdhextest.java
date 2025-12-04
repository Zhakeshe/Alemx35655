package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class hdhextest extends OpMode {

    private DcMotor motorTest;


    @Override
    public void init() {
        motorTest = hardwareMap.get(DcMotor.class, "motorTest");
    }

    @Override
    public void loop() {
        double rt = gamepad1.right_trigger;
        double lt = gamepad1.left_trigger;
        double dd = Math.max(Math.abs(rt) + Math.abs(lt), 1);
        motorTest.setPower(((rt - lt) / dd) * 3);

    }
}
