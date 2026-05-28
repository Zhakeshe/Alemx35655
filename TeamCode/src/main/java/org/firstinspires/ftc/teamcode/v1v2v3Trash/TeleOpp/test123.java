package org.firstinspires.ftc.teamcode.v1v2v3Trash.TeleOpp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@Disabled
public class test123 extends OpMode {

    private DcMotor lfex0;
    private DcMotor lrex1;
    private DcMotor rfex2;
    private DcMotor rrex3;

    @Override
    public void init() {
        lfex0 = hardwareMap.get(DcMotor.class, "lfex0");
        lrex1 = hardwareMap.get(DcMotor.class, "lrex1");
        rfex2 = hardwareMap.get(DcMotor.class, "rfex2");
        rrex3 = hardwareMap.get(DcMotor.class, "rrex3");

        lfex0.setDirection(DcMotorSimple.Direction.REVERSE);
        lrex1.setDirection(DcMotorSimple.Direction.FORWARD);
        rfex2.setDirection(DcMotorSimple.Direction.FORWARD);
        rrex3.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void loop() {
        if (gamepad1.a){
            lfex0.setPower(1000);

        } else if (gamepad1.b) {
            lrex1.setPower(1000);
        } else if (gamepad1.y) {
            rfex2.setPower(1000);
        } else if (gamepad1.x) {
            rrex3.setPower(1000);
        }else{
            lfex0.setPower(0);
            lrex1.setPower(0);
            rfex2.setPower(0);
            rrex3.setPower(0);
        }

    }
}
