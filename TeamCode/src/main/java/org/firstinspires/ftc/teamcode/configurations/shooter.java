package org.firstinspires.ftc.teamcode.configurations;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class shooter {
    private DcMotorEx Outtake1;
    private DcMotorEx Outtake2;

    boolean tfout1 = false;
    boolean tfout2 = true;
    boolean tfout3 = false;
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

    public static double ref = 1700;
    public static double ref1 = 1300;

    //

    double integralSum3 = 0;

    public static double Kp3 = 0.01;
    public static double Ki3 = 0;
    public static double Kd3 = 0.0001;

    public static double Kf3 = 0.95;
    ElapsedTime timer3 = new ElapsedTime();
    private double lastError3 = 0;

    //

    double integralSum4 = 0;

    public static double Kp4 = 0.01;
    public static double Ki4 = 0;
    public static double Kd4 = 0.0001;

    public static double Kf4 = 0.95;
    ElapsedTime timer4 = new ElapsedTime();
    private double lastError4 = 0;

    public void init(HardwareMap hwMap){
        Outtake1 = hwMap.get(DcMotorEx.class, "Outtake1");
        Outtake2 = hwMap.get(DcMotorEx.class, "Outtake2");

        Outtake1.setDirection(DcMotorSimple.Direction.REVERSE);
        Outtake2.setDirection(DcMotorSimple.Direction.FORWARD);

        Outtake1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Outtake2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Outtake1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Outtake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void shoot(boolean g2b, Gamepad gamepad2){
        if(g2b & !tfout1){
            tfout2 = false;
            if (Outtake1.getPower() == 0){
                tfout1 = true;
                tfout2 = true;
                Outtake1.setVelocity(PIDcontrollLow1((ref1), (Outtake1.getVelocity())));
                Outtake2.setVelocity(PIDcontrollLow2((ref1), (Outtake2.getVelocity())));
                gamepad2.rumble(0.5, 0.5, 10000000);
                tfout3 = false;
            } else if (Outtake1.getPower() > 0 && !tfout3) {
                tfout1 = true;
                tfout2 = true;
                Outtake1.setVelocity(PIDcontrollHigh1((ref), (Outtake1.getVelocity())));
                Outtake2.setVelocity(PIDcontrollHigh2((ref), (Outtake2.getVelocity())));
                gamepad2.rumble(1.0, 1.0, 10000000);
                tfout3 = true;
            } else if (tfout3) {
                tfout1 = true;
                tfout2 = true;
                Outtake1.setPower(0);
                Outtake2.setPower(0);
                gamepad2.stopRumble();
                tfout3 = false;
            }
        }
        else if (!gamepad2.b & tfout2) {
            tfout1 = false;
        }
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

    public double PIDcontrollLow1(double reference3, double state3){
        double error3 = reference3 - state3;
        integralSum3 += error3 * timer3.seconds();
        double derivative3 = (error3 - lastError3) / timer3.seconds();
        lastError3 = error3;

        timer3.reset();

        return (error3 * Kp3) + (derivative3 * Kd3) + (integralSum3 * Ki3) + (reference3 * Kf3);


    }
    public double PIDcontrollLow2(double reference4, double state4){
        double error4 = reference4 - state4;
        integralSum4 += error4 * timer4.seconds();
        double derivative4 = (error4 - lastError4) / timer4.seconds();
        lastError4 = error4;

        timer4.reset();

        return (error4 * Kp4) + (derivative4 * Kd4) + (integralSum4 * Ki4) + (reference4 * Kf4);


    }
}
