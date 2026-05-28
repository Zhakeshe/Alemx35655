package org.firstinspires.ftc.teamcode.v1v2v3Trash.TeleOpp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


@Disabled
@Config

public class pid2 extends LinearOpMode {

    DcMotorEx Outtake1;
    DcMotorEx Outtake2;

    double integralSum1 = 0;
    double Kp1 = 0;
    double Ki1 = 0;
    double Kd1 = 0;
    double Kf1 = 1;
    ElapsedTime timer1 = new ElapsedTime();
    private double lastError1 = 0;

    double integralSum2 = 0;
    double Kp2 = 0;
    double Ki2 = 0;
    double Kd2 = 0;
    double Kf2 = 5;
    ElapsedTime timer2 = new ElapsedTime();
    private double lastError2 = 0;

    private double outpu1 = 0;
    private double output2 = 0;

    double target = 1000;

    TelemetryPacket packet = new TelemetryPacket();
    FtcDashboard dashboard = FtcDashboard.getInstance();



    @Override
    public void runOpMode() throws InterruptedException {

        Outtake1 = hardwareMap.get(DcMotorEx.class, "Outtake1");
        Outtake2 = hardwareMap.get(DcMotorEx.class, "Outtake2");

        Outtake1.setDirection(DcMotorSimple.Direction.FORWARD);
        Outtake2.setDirection(DcMotorSimple.Direction.REVERSE);

        Outtake1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Outtake2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        while (opModeIsActive()){
            double power1 = PIDcontrollForOut1(target, Outtake1.getVelocity());
            double power2 = PIDcontrollForOut2(target, Outtake2.getVelocity());
            Outtake1.setPower(power1);
            Outtake2.setPower(power2);


            packet.put("Target", target);
            packet.put("Velocity1", Outtake1.getVelocity());
            packet.put("Velocity2", Outtake2.getVelocity());
            packet.put("Power1", power1);
            packet.put("Power2", power2);
            dashboard.sendTelemetryPacket(packet);
        }
    }



    public double PIDcontrollForOut1(double reference1, double state1){
        double error1 = reference1 - state1;
        integralSum1 += error1 * timer1.seconds();
        double derivative1 = (error1 - lastError1) / timer1.seconds();
        lastError1 = error1;

        timer1.reset();

        double output1 = (error1 * Kp1) + (derivative1 * Kd1) + (integralSum1 * Ki1) + (reference1 * Kf1);
        packet.put("out1", output1);
        return output1;


    }

    public double PIDcontrollForOut2(double reference2, double state2){
        double error2 = reference2 - state2;
        integralSum2 += error2 * timer2.seconds();
        double derivative2 = (error2 - lastError2) / timer2.seconds();
        lastError2 = error2;

        timer2.reset();

        double output2 = (error2 * Kp2) + (derivative2 * Kd2) + (integralSum2 * Ki2) + (reference2 * Kf2);
        packet.put("out2", output2);
        return output2;


    }
}
