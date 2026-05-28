package org.firstinspires.ftc.teamcode.v1v2v3Trash.TeleOpp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebCam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Disabled
public class RedMainCode extends OpMode {

    AprilTagWebCam aprilTagWebCam = new AprilTagWebCam();

    private DcMotor Intake0;

    private DcMotorEx Outtake1;
    private DcMotorEx Outtake2;

    private Servo z2;

    private Servo o4;
    private Servo o5;

    private DcMotor lfex0;
    private DcMotor lrex1;
    private DcMotor rfex2;
    private DcMotor rrex3;

    final double kp = 0.0045;
    boolean tftrans = false;
    boolean tftrans1 = true;
    boolean tfout1 = false;
    boolean tfout2 = true;
    boolean tfcam = true;

    int i1 = 0;

    double integralSum1 = 0;

    public static double Kp1 = 0.004;
    public static double Ki1 = 0;
    public static double Kd1 = 0.001;

    public static double Kf1 = 0.01;
    ElapsedTime timer1 = new ElapsedTime();
    private double lastError1 = 0;

    double integralSum2 = 0;

    public static double Kp2 = 0.004;
    public static double Ki2 = 0;
    public static double Kd2 = 0.001;

    public static double Kf2 = 0.01;
    ElapsedTime timer2 = new ElapsedTime();
    private double lastError2 = 0;

    int ref = 1500;





    @Override
    public void init() {
        aprilTagWebCam.init(hardwareMap, telemetry);


        Intake0 = hardwareMap.get(DcMotor.class, "Intake0");

        Outtake1 = hardwareMap.get(DcMotorEx.class, "Outtake1");
        Outtake2 = hardwareMap.get(DcMotorEx.class, "Outtake2");

        z2 = hardwareMap.get(Servo.class, "z2");


        o4 = hardwareMap.get(Servo.class, "o4");
        o5 = hardwareMap.get(Servo.class, "o5");

        lfex0 = hardwareMap.get(DcMotor.class, "lfex0");
        lrex1 = hardwareMap.get(DcMotor.class, "lrex1");
        rfex2 = hardwareMap.get(DcMotor.class, "rfex2");
        rrex3 = hardwareMap.get(DcMotor.class, "rrex3");


        Outtake1.setDirection(DcMotorSimple.Direction.FORWARD);
        Outtake2.setDirection(DcMotorSimple.Direction.REVERSE);

        Intake0.setDirection(DcMotorSimple.Direction.REVERSE);

        lfex0.setDirection(DcMotorSimple.Direction.FORWARD);
        lrex1.setDirection(DcMotorSimple.Direction.REVERSE);
        rfex2.setDirection(DcMotorSimple.Direction.FORWARD);
        rrex3.setDirection(DcMotorSimple.Direction.REVERSE);

        Outtake1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Outtake2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Outtake1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Outtake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        o4.setPosition(0.90);
        o5.setPosition(0);

    }



    @Override
    public void loop() {
        Outtake();
        OutCam();
        Intake();
        meka();
        Trnans();

    }



    public void meka() {
        double yy = gamepad1.right_trigger;
        double qy = -gamepad1.left_trigger;
        double turn = gamepad1.left_stick_x;
        double x = -gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(qy) + Math.abs(yy) + Math.abs(x) + Math.abs(turn), 1);

        lfex0.setPower((qy + yy + x + turn) / denominator);
        lrex1.setPower((qy + yy - x + turn) / denominator);
        rfex2.setPower((qy + yy - x - turn) / denominator);
        rrex3.setPower((qy + yy + x - turn) / denominator);
    }



    public void Outtake() {
        aprilTagWebCam.update();
        AprilTagDetection tag24 = aprilTagWebCam.getTagSpecificId(24);
        aprilTagWebCam.displayDetectionTelemetry(tag24);

        if (gamepad2.b & !tfout1) {
            tfout2 = false;
            if (Outtake1.getPower() > 0) {
                tfout1 = true;
                Outtake1.setPower(0);
                Outtake2.setPower(0);
                tfout2 = true;
            } else if (Outtake1.getPower() == 0) {
                tfout1 = true;
                if(tag24 != null && tfcam){
                    double camy = tag24.ftcPose.y;
                    if (camy <= 110){
                        Outtake1.setPower((PIDcontrollForOut1(ref, Outtake1.getVelocity())) * 60);
                        Outtake2.setPower((PIDcontrollForOut2(ref, Outtake2.getVelocity())) * 60);
                        tfout2 = true;
                        tfcam = false;
                        telemetry.addData("qwe", (PIDcontrollForOut1(ref, Outtake1.getVelocity())));
                        telemetry.update();
                    }
                    else if (111 < camy && camy <= 170 ){
                        Outtake1.setPower((PIDcontrollForOut1(ref, Outtake1.getVelocity())) * 120);
                        Outtake2.setPower((PIDcontrollForOut2(ref, Outtake2.getVelocity())) * 120);
                        tfout2 = true;
                        tfcam = false;
                        telemetry.addData("qwe", (PIDcontrollForOut1(ref, Outtake1.getVelocity())));
                        telemetry.update();
                    }else if(171 < camy && camy <= 230){
                        Outtake1.setPower((PIDcontrollForOut1(ref, Outtake1.getVelocity())) * 180);
                        Outtake2.setPower((PIDcontrollForOut2(ref, Outtake2.getVelocity())) * 180);
                        tfout2 = true;
                        tfcam = false;
                        telemetry.addData("qwe", (PIDcontrollForOut1(ref, Outtake1.getVelocity())));
                        telemetry.update();
                    }else if(231 < camy && camy <= 300){
                        Outtake1.setPower((PIDcontrollForOut1(ref, Outtake1.getVelocity())) * 230);
                        Outtake2.setPower((PIDcontrollForOut2(ref, Outtake2.getVelocity())) * 230);
                        tfout2 = true;
                        tfcam = false;
                        telemetry.addData("qwe", (PIDcontrollForOut1(ref, Outtake1.getVelocity())));
                        telemetry.update();
                    } else if (301 < camy) {
                        Outtake1.setPower((PIDcontrollForOut1(ref, Outtake1.getVelocity())) * 285);
                        Outtake2.setPower((PIDcontrollForOut2(ref, Outtake2.getVelocity())) * 285);
                        tfout2 = true;
                        tfcam = false;
                        telemetry.addData("qwe", (PIDcontrollForOut1(ref, Outtake1.getVelocity())));
                        telemetry.update();
                    }
                }else{
                    Outtake1.setPower((PIDcontrollForOut1(ref, Outtake1.getVelocity())) * 285);
                    Outtake2.setPower((PIDcontrollForOut2(ref, Outtake2.getVelocity())) * 285);
                    tfout2 = true;
                    tfcam = false;
                    telemetry.addData("qwe", (PIDcontrollForOut1(ref, Outtake1.getVelocity())));
                    telemetry.update();
                }
            }
        }else if (!gamepad2.b && tfout2) {
            tfout1 = false;
            tfcam = true;
        }

        telemetry.update();
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
        lastError1 = error2;

        timer1.reset();

        double output2 = (error2 * Kp2) + (derivative2 * Kd2) + (integralSum1 * Ki2) + (reference2 * Kf2);
        return output2;


    }





    public void Intake() {
        double rt2 = gamepad2.right_trigger;
        double lt2 = gamepad2.left_trigger;

        double denominator = Math.max(Math.abs(rt2) + Math.abs(rt2), 1);
        Intake0.setPower(((rt2 * 2)- (lt2 / 2) / denominator));

    }



    public void Trnans() {
        if (gamepad1.a & !tftrans) {
            tftrans1 = false;
            if (o5.getPosition() == 0) {
                o5.setPosition(0.7);
                tftrans = true;
                tftrans1 = true;
                i1 = 0;

            } else if (o5.getPosition() > 0) {
                o5.setPosition(0);
                tftrans = true;
                tftrans1 = true;
            }
        } else if (!gamepad1.a && tftrans1) {
            tftrans = false;
        }
    }



    public void OutCam(){
        aprilTagWebCam.update();
        AprilTagDetection tag24 = aprilTagWebCam.getTagSpecificId(24);
        aprilTagWebCam.displayDetectionTelemetry(tag24);

        if(tag24 != null){
            double camy = tag24.ftcPose.y;
            double z2pos = (0.6 - (camy * kp));
            z2.setPosition(z2pos);

        }
        telemetry.update();
    }
}
