package org.firstinspires.ftc.teamcode.mainCode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp
public class CaMainBlue extends OpMode {

    private Servo p1; // 0 exp
    private Servo p2; // 0 con

    private Servo xr; // 1 exp
    private Servo xl; // 1 com

    private Limelight3A limelight;

    private DcMotorEx Outtake1; // 0 con
    private DcMotorEx Outtake2; // 0 exp

    private DcMotor Intake; // 1 con
    private DcMotor Trans; // 1 exp

    private DcMotor lf; // 2 con
    private DcMotor lr; // 3 con
    private DcMotor rf; // 2 exp
    private DcMotor rr; // 3 exp


    //###############TF

    boolean tfout1 = false;
    boolean tfout2 = true;
    boolean tfout3 = false;

    boolean tfpark1 = false;
    boolean tfpark2 = true;

    boolean tfin1 = false;
    boolean tfin2 = true;

    boolean tftrans1 = false;
    boolean tftrans2 = true;


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


    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8);
        limelight.start();

        p1 = hardwareMap.get(Servo.class, "p1");
        p2 = hardwareMap.get(Servo.class, "p2");

        xl = hardwareMap.get(Servo.class, "xl");
        xr = hardwareMap.get(Servo.class, "xr");

        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Trans = hardwareMap.get(DcMotor.class, "Trans");

        Outtake1 = hardwareMap.get(DcMotorEx.class, "Outtake1");
        Outtake2 = hardwareMap.get(DcMotorEx.class, "Outtake2");

        lf = hardwareMap.get(DcMotor.class, "lf");
        lr = hardwareMap.get(DcMotor.class, "lr");
        rf = hardwareMap.get(DcMotor.class, "rf");
        rr = hardwareMap.get(DcMotor.class, "rr");

        xl.setPosition(0.5);
        xr.setPosition(0.5);


        Intake.setDirection(DcMotorSimple.Direction.REVERSE);
        Trans.setDirection(DcMotorSimple.Direction.FORWARD);

        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lr.setDirection(DcMotorSimple.Direction.REVERSE);
        rf.setDirection(DcMotorSimple.Direction.FORWARD);
        rr.setDirection(DcMotorSimple.Direction.FORWARD);

        Outtake1.setDirection(DcMotorSimple.Direction.REVERSE);
        Outtake2.setDirection(DcMotorSimple.Direction.FORWARD);

        Outtake1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Outtake2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Outtake1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Outtake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        Outtake();
        Turel();
        Park();
        Intake();
        strafe();


    }

    public void strafe() {
        double yy = gamepad1.right_trigger;
        double qy = -gamepad1.left_trigger;
        double turn = gamepad1.left_stick_x;
        double x = -gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(qy) + Math.abs(yy) + Math.abs(x) + Math.abs(turn), 1);

        lf.setPower((qy + yy + x + turn) / denominator);
        lr.setPower((qy + yy - x + turn) / denominator);
        rf.setPower((qy + yy - x - turn) / denominator);
        rr.setPower((qy + yy + x - turn) / denominator);
    }


    public void Intake(){
        if(gamepad1.a && !tfin1){
            tfin2 = false;
            if(Intake.getPower() == 0){
                tfin1 = true;

                tfin2 = true;
                Intake.setPower(1);
                Trans.setPower(-1);

            } else if (Intake.getPower() > 0) {
                tfin1 = true;
                tfin2 = true;
                Intake.setPower(0);
                Trans.setPower(0);
            }
        }
        else if(!gamepad1.a && tfin2){
            tfin1 = false;
        }

        if (gamepad2.a && !tftrans1){
            tftrans2 = false;
            if (Trans.getPower() > 0){
                tftrans1 = true;
                tftrans2 = true;
                Trans.setPower(-1);
            } else if (Trans.getPower() < 0) {
                tftrans1 = true;
                tftrans2 = true;
                Trans.setPower(1);
            }
        }
        else if (!gamepad2.a && tftrans2) {
            tftrans1 = false;
        }
    }

    public void Park(){
        if (gamepad1.y & !tfpark1){
            tfpark2 = false;
            if(p1.getPosition() == 0){
                tfpark1 = true;
                tfpark2 = true;
                p1.setPosition(0.5);
                p2.setPosition(0.5);
            } else if (p1.getPosition() == 0.5) {
                tfpark1 = true;
                tfpark2 = true;
                p1.setPosition(0);
                p2.setPosition(1);
            }
        } else if (!gamepad1.y && tfpark2) {
            tfpark1 = false;
        }
    }

    public void Turel(){
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()){
            telemetry.addData("Tx", llResult.getTx());
            double pos = 0.5 - llResult.getTx() * 0.0115;
            pos = Range.clip(pos, 0.0, 1.0);
            xr.setPosition(pos);
            xl.setPosition(pos);

        }
    }

    public void Outtake(){
        if(gamepad2.b & !tfout1){
            tfout2 = false;
            if (Outtake1.getPower() == 0){
                tfout1 = true;
                tfout2 = true;
                Outtake1.setVelocity(PIDcontrollLow1((ref1), (Outtake1.getVelocity())));
                Outtake2.setVelocity(PIDcontrollLow2((ref1), (Outtake2.getVelocity())));
                gamepad2.rumble(0.5, 0.5, 10000);
                tfout3 = false;
            } else if (Outtake1.getPower() > 0 && !tfout3) {
                tfout1 = true;
                tfout2 = true;
                Outtake1.setVelocity(PIDcontrollHigh1((ref), (Outtake1.getVelocity())));
                Outtake2.setVelocity(PIDcontrollHigh2((ref), (Outtake2.getVelocity())));
                gamepad2.rumble(1.0, 1.0, 10000);
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



    //#######################################################################################################################################################
    //##########################PID##########################################################################################################################

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
