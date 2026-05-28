package org.firstinspires.ftc.teamcode.configTamaraV3;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class shooterAndIntake {

    private DcMotorEx shooter1, shooter2;
    private DcMotor intake, transfer;

    //private Servo blocker;

    boolean sht1 = false;
    boolean sht2 = false;
    boolean sht3 = false;

    boolean tt1 = false;
    boolean tt2 = false;


    //##########################PID1
    double integralSum1 = 0;

    public static double Kp1 = 0.01;
    public static double Ki1 = 0;
    public static double Kd1 = 0.0005;

    public static double Kf1 = 0.95;
    ElapsedTime timer1 = new ElapsedTime();
    private double lastError1 = 0;

    //##########################PID2
    double integralSum2 = 0;

    public static double Kp2 = 0.01;
    public static double Ki2 = 0;
    public static double Kd2 = 0.0005;

    public static double Kf2 = 0.95;
    ElapsedTime timer2 = new ElapsedTime();
    private double lastError2 = 0;

    public static double ref = 1800;

    //


    public void init(HardwareMap hwMap){
        intake = hwMap.get(DcMotor.class, "intake");
        transfer = hwMap.get(DcMotor.class, "transfer");
        shooter1 = hwMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hwMap.get(DcMotorEx.class, "shooter2");

        //blocker = hardwareMap.get(Servo.class, "blocker");

        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        transfer.setDirection(DcMotorSimple.Direction.FORWARD);

        shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void shooterIntake(boolean grb, boolean glb, boolean g1a){

        shooter1.setVelocity(PIDcontrollHigh1((ref), (shooter1.getVelocity())));
        shooter2.setVelocity(PIDcontrollHigh2((ref), (shooter2.getVelocity())));

        if (grb && !sht1){
            sht2 = true;
            if (intake.getPower() == 0){
                sht1 = true;
                sht2 = true;
                sht3 = false;
                intake.setPower(1);
                transfer.setPower(1);
                ref = 1300;
            }else if(intake.getPower() > 0 && !sht3){
                sht1 = true;
                sht2 = true;
                sht3 = true;
                intake.setPower(1);
                transfer.setPower(1);
                ref = 1800;
            }
            else if(sht3){
                sht1 = true;
                sht2 = true;
                sht3 = false;
                intake.setPower(0);
                transfer.setPower(0);
                ref = 0;

            }
        } else if (glb) {
            sht1 = true;
            sht2 = true;
            sht3 = true;
            intake.setPower(0);
            transfer.setPower(0);
            ref = 0;
        }else if (!grb & sht2) {
            sht1 = false;
        }
/*
        if (g1a && !tt1){
            tt2 = false;
            if (blocker.getPosition() == 0){
                tt1 = true;
                tt2 = true;
                blocker.setPosition(1);
            }else {
                tt1 = true;
                tt2 = true;
                blocker.setPosition(0);
            }
        } else if (!g1a && tt2) {
            tt1 = false;
        }
*/
    }


    public double PIDcontrollHigh1(double reference1, double state1){
        double error1 = reference1 - state1;
        integralSum1 += error1 * timer1.seconds();
        double derivative1 = (error1 - lastError1) / timer1.seconds();
        lastError1 = error1;

        timer1.reset();

        return (error1 * Kp1) + (derivative1 * Kd1) + (integralSum1 * Ki1) + (reference1 * Kf1);


    }
    public double PIDcontrollHigh2(double reference2, double state2){
        double error2 = reference2 - state2;
        integralSum2 += error2 * timer2.seconds();
        double derivative2 = (error2 - lastError2) / timer2.seconds();
        lastError2 = error2;

        timer2.reset();

        return (error2 * Kp2) + (derivative2 * Kd2) + (integralSum2 * Ki2) + (reference2 * Kf2);


    }
}
