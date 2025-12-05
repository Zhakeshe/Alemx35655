package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebCam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;         

@TeleOp
public class testInOut extends OpMode {

    AprilTagWebCam aprilTagWebCam = new AprilTagWebCam();

    private DcMotor Intake0;

    private DcMotorEx Outtake1;
    private DcMotorEx Outtake2;

    private Servo xy0;
    private Servo xy1;

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

    boolean tfin = false;
    boolean tfin2 = true;
    boolean tfin3 = false;
    boolean tfin4 = true;

    boolean tfout1 = false;
    boolean tfout2 = true;

    int i1 = 0;

    double integralSum1 = 0;

    public static double Kp1 = 10;
    public static double Ki1 = 0;
    public static double Kd1 = 10;
    public static double Kf1 = 10;
    ElapsedTime timer1 = new ElapsedTime();
    private double lastError1 = 0;

    public static double Kp2 = 10;
    public static double Ki2 = 0;
    public static double Kd2 = 10;
    public static double Kf2 = 10;
    ElapsedTime timer2 = new ElapsedTime();

    private double lastError2 = 0;
    double integralSum2 = 0;



    @Override
    public void init() {
        aprilTagWebCam.init(hardwareMap, telemetry);


        Intake0 = hardwareMap.get(DcMotor.class, "Intake0");

        Outtake1 = hardwareMap.get(DcMotorEx.class, "Outtake1");
        Outtake2 = hardwareMap.get(DcMotorEx.class, "Outtake2");

        xy0 = hardwareMap.get(Servo.class, "xy0");
        xy1 = hardwareMap.get(Servo.class, "xy1");

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

        Outtake1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Outtake2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        o4.setPosition(0.96);
        o5.setPosition(0);

    }



    @Override
    public void loop() {
        Outtake();
        OutCam();
        Intake();
        meka();
        Trnans();
        xy();


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

        if (gamepad2.b & !tfout1) {
            tfout2 = false;
            if (Outtake1.getPower() == 1) {
                tfout1 = true;
                Outtake1.setVelocity(0);
                Outtake2.setVelocity(0);
                tfout2 = true;
            } else if (Outtake1.getPower() == 0) {
                tfout1 = true;
                Outtake1.setVelocity(PIDcontrollForOut1(6000, Outtake1.getVelocity()));
                Outtake2.setVelocity(PIDcontrollForOut2(6000, Outtake2.getVelocity()));
                tfout2 = true;
            }
        }else if (!gamepad2.b && tfout2) {
            tfout1 = false;
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





    public void Intake() {

        if (gamepad2.right_bumper & !tfin) {
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
        } else if (!gamepad2.right_bumper && tfin2) {
            tfin = false;
        }

        if (gamepad2.left_bumper & ! tfin3 && Intake0.getPower() <= 0) {
            tfin4 = false;
            if (Intake0.getPower() == 0){
                Intake0.setPower(-50);
                tfin3 = true;
                tfin4 = true;
            } else if (Intake0.getPower() < 0) {
                Intake0.setPower(0);
                tfin3 = true;
                tfin4 = true;
            }
        }else if (!gamepad2.left_bumper && tfin4) {
            tfin3 = false;
        }

    }



    public void Trnans() {
        if (gamepad1.a & !tftrans) {
            tftrans1 = false;
            if (o4.getPosition() == 0.96) {
                o4.setPosition(0.75);
                tftrans = true;
                for(i1=1; i1<27000000; i1++){}
                if (o4.getPosition() == 0.75 && i1 == 27000000) {
                    o5.setPosition(0.7);
                    tftrans1 = true;
                    i1 = 0;
                }
            } else if (o4.getPosition() == 0.75) {
                o4.setPosition(0.96);
                tftrans = true;
                if (o4.getPosition() == 0.96 ) {
                    o5.setPosition(0);
                    tftrans1 = true;
                }
            }
        } else if (!gamepad1.a && tftrans1) {
            tftrans = false;
        }
    }



    public void OutCam(){
        aprilTagWebCam.update();
        AprilTagDetection tag = aprilTagWebCam.getTagSpecificId(24);
        aprilTagWebCam.displayDetectionTelemetry(tag);

        if(tag != null){
            double camy = tag.ftcPose.y;
            double z2pos = (0.6 - (camy * kp));
            z2.setPosition(z2pos);

        }
        telemetry.update();
    }
    public void xy(){
        double xrrr = -gamepad2.right_stick_x;
        xy0.setPosition(0.5 + xrrr);
        xy1.setPosition(0.5 + xrrr);
    }
}
