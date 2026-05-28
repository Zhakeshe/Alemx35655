package org.firstinspires.ftc.teamcode.v1v2v3Trash.configurations;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class strafe {

    private DcMotor lf,lr,rf,rr;


    public void init(HardwareMap hwMap) {
        lf = hwMap.get(DcMotor.class, "lf");
        lr = hwMap.get(DcMotor.class, "lr");
        rf = hwMap.get(DcMotor.class, "rf");
        rr = hwMap.get(DcMotor.class, "rr");

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lr.setDirection(DcMotorSimple.Direction.REVERSE);
        rf.setDirection(DcMotorSimple.Direction.FORWARD);
        rr.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void baseStrafe(double yy, double qy, double turn, double x){
        double denominator = Math.max(Math.abs(qy) + Math.abs(yy) + Math.abs(x) + Math.abs(turn), 1);

        lf.setPower((qy + yy + x + turn) / denominator);
        lr.setPower((qy + yy - x + turn) / denominator);
        rf.setPower((qy + yy - x - turn) / denominator);
        rr.setPower((qy + yy + x - turn) / denominator);
    }

}
