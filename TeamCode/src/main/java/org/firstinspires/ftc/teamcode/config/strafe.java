package org.firstinspires.ftc.teamcode.config;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class strafe{



    private DcMotor lf, rf, lr, rr;


    public void init(HardwareMap hardwareMap){
        lf = hardwareMap.get(DcMotor.class, "lf"); //2
        rf = hardwareMap.get(DcMotor.class, "rf"); //0
        lr = hardwareMap.get(DcMotor.class, "lr"); //3
        rr = hardwareMap.get(DcMotor.class, "rr"); //1

        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lr.setDirection(DcMotorSimple.Direction.REVERSE);
        rf.setDirection(DcMotorSimple.Direction.FORWARD);
        rr.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void baseStrafe(double yy, double qy, double turn, double x) {
        double denominator = Math.max(Math.abs(qy) + Math.abs(yy) + Math.abs(x) + Math.abs(turn), 1);

        lf.setPower((qy + yy + x + turn) / denominator);
        lr.setPower((qy + yy - x + turn) / denominator);
        rf.setPower((qy + yy - x - turn) / denominator);
        rr.setPower((qy + yy + x - turn) / denominator);
    }
}
