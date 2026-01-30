package org.firstinspires.ftc.teamcode.TeleOpp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class pid2motor extends OpMode {

    private DcMotorEx Outtake1;
    private DcMotorEx Outtake2;

    //##########################PID1
    double integralSum1 = 0;

    public static double Kp1 = 2;
    public static double Ki1 = 0.1;
    public static double Kd1 = 0.3;

    public static double Kf1 = 0.1;
    ElapsedTime timer1 = new ElapsedTime();
    private double lastError1 = 0;

    //##########################PID2
    double integralSum2 = 0;

    public static double Kp2 = 2;
    public static double Ki2 = 0.1;
    public static double Kd2 = 0.3;

    public static double Kf2 = 0.1;
    ElapsedTime timer2 = new ElapsedTime();
    private double lastError2 = 0;

    public static double ref = 2700;

    final double kp = 0.0045;


    boolean tfout1 = false;
    boolean tfout2 = true;

    private int ref1 = 1000;
    private int ref2 = 1000;

    @Override
    public void init() {
        Outtake1 = hardwareMap.get(DcMotorEx.class, "Outtake1");
        Outtake2 = hardwareMap.get(DcMotorEx.class, "Outtake2");

        Outtake1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Outtake2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Outtake1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Outtake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Outtake1.setDirection(DcMotorSimple.Direction.FORWARD);
        Outtake2.setDirection(DcMotorSimple.Direction.REVERSE);


    }

    @Override
    public void loop() {
        Outtake();

    }

    public double PIDcontrollForOut1(double reference1, double state1){
        double error1 = reference1 - state1;
        integralSum1 += error1 * timer1.seconds();
        double derivative1 = (error1 - lastError1) / timer1.seconds();
        lastError1 = error1;

        timer1.reset();

        double output1 = (error1 * Kp1) + (derivative1 * Kd1) + (integralSum1 * Ki1) + (reference1 * Kf1);
        return output1;

    }


    public void Outtake(){
        if (gamepad2.b & !tfout1) {
            tfout2 = false;
            if (Outtake1.getPower() > 0) {
                tfout1 = true;
                Outtake1.setPower(0);
                Outtake2.setPower(0);
                tfout2 = true;
            } else if (Outtake1.getPower() == 0) {
                tfout1 = true;
                Outtake1.setPower(6000);
                Outtake2.setPower(6000);
                telemetry.addData("1", Outtake1.getPower());
                telemetry.addData("2", Outtake2.getPower());
                telemetry.update();
                tfout2 = true;
            }
        }else if (!gamepad2.b && tfout2) {
            tfout1 = false;
        }
    }


}
