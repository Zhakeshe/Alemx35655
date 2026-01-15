package org.firstinspires.ftc.teamcode.TeleOpp;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.FtcDashboard;

@Disabled
@Config

public class pid extends LinearOpMode {

    DcMotorEx Outtake1;
    DcMotorEx Outtake2;

    double integralSum1 = 0;

    public static double Kp1 = 0.004;
    public static double Ki1 = 0;
    public static double Kd1 = 0.001;

    public static double Kf1 = 0.01;
    ElapsedTime timer1 = new ElapsedTime();
    private double lastError1 = 0;

    double integralSum2 = 0;
    public static double Kp2 = 10;
    public static double Ki2 = 0;
    public static double Kd2 = 10;
    public static double Kf2 = 10;
    ElapsedTime timer2 = new ElapsedTime();
    private double lastError2 = 0;

    private DcMotor Intake0;

    private Servo o4;
    private Servo o5;

    boolean tftrans = false;
    boolean tftrans1 = true;

    boolean tfin = false;
    boolean tfin2 = true;

    boolean tfout1 = false;
    boolean tfout2 = true;
    int i1 = 0;
    TelemetryPacket packet = new TelemetryPacket();
    FtcDashboard dashboard = FtcDashboard.getInstance();

    int tps = 1500;




    @Override
    public void runOpMode() throws InterruptedException {

        Outtake1 = hardwareMap.get(DcMotorEx.class, "Outtake1");
        Outtake2 = hardwareMap.get(DcMotorEx.class, "Outtake2");

        Intake0 = hardwareMap.get(DcMotor.class, "Intake0");

        o4 = hardwareMap.get(Servo.class, "o4");
        o5 = hardwareMap.get(Servo.class, "o5");

        Outtake1.setDirection(DcMotorSimple.Direction.FORWARD);
        Outtake2.setDirection(DcMotorSimple.Direction.REVERSE);

        Intake0.setDirection(DcMotorSimple.Direction.REVERSE);

        Outtake1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Outtake1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Outtake1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Outtake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        o4.setPosition(0.80);
        o5.setPosition(0);



        waitForStart();
        while (opModeIsActive()){
            /*double power1 = PIDcontrollForOut1(1000, Outtake1.getVelocity());
            double power2 = PIDcontrollForOut2(1000, Outtake2.getVelocity());
            Outtake1.setPower(power1);
            Outtake2.setPower(power2);
*/

            Trnans();
            Intake();
            Outtake();

        }
    }

    public void Outtake() {

        if (gamepad2.b & !tfout1) {
            tfout2 = false;
            if (Outtake1.getPower() > 0) {
                tfout1 = true;
                Outtake1.setPower(0);
                Outtake2.setPower(0);
                tfout2 = true;
            } else if (Outtake1.getPower() == 0) {
                tfout1 = true;
                Outtake1.setPower(PIDcontrollForOut1(tps, Outtake1.getVelocity()));
                Outtake2.setPower(PIDcontrollForOut1(tps, Outtake2.getVelocity()));
                tfout2 = true;
            }
        }else if (!gamepad2.b && tfout2) {
            tfout1 = false;
        }
    }



    public void Intake() {

        if (gamepad2.a & !tfin) {
            tfin2 = false;
            if (Intake0.getPower() == 0) {
                Intake0.setPower(6000);
                tfin = true;
                tfin2 = true;
            } else if (Intake0.getPower() == 1) {
                Intake0.setPower(0);
                tfin = true;
                tfin2 = true;
            }
        } else if (!gamepad2.a && tfin2) {
            tfin = false;
        }
    }

    public void Trnans() {
        if (gamepad1.a & !tftrans) {
            tftrans1 = false;
            if (o5.getPosition() == 0) {
                o5.setPosition(0.7);
                tftrans = true;
                tftrans1 = true;
                i1 = 0;

            } else if (o5.getPosition() == 0.7) {
                o5.setPosition(0);
                tftrans = true;
                tftrans1 = true;
            }
        } else if (!gamepad1.a && tftrans1) {
            tftrans = false;
        }
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

    public double PIDcontrollForOut2(double reference2, double state2){
        double error2 = reference2 - state2;
        integralSum2 += error2 * timer2.seconds();
        double derivative2 = (error2 - lastError2) / timer2.seconds();
        lastError2 = error2;

        timer2.reset();


        double output2 = (error2 * Kp2) + (derivative2 * Kd2) + (integralSum2 * Ki2) + (reference2 * Kf2);
        return output2;


    }
}
