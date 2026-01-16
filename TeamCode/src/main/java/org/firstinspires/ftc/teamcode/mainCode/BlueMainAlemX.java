package org.firstinspires.ftc.teamcode.mainCode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebCam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp
public class BlueMainAlemX extends OpMode {
    AprilTagWebCam aprilTagWebCam = new AprilTagWebCam();

    private DcMotor lfex0;
    private DcMotor lrex1;
    private DcMotor rfex2;
    private DcMotor rrex3;

    private DcMotor Intake0ex;

    private DcMotorEx Outtake1ex;
    private DcMotorEx Outtake2ex;

    private DcMotor Trans3ex;

    private Servo z0;
    private Servo z1;


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

    //##########################TF
    boolean tfout1 = false;
    boolean tfout2 = true;

    boolean tfin1 = false;
    boolean tfin2 = true;

    boolean tftrans1 = false;
    boolean tftrans2 = true;
    boolean tftrans = true;
    double camy = 1;


    @Override
    public void init() {
        aprilTagWebCam.init(hardwareMap, telemetry);

        lfex0 = hardwareMap.get(DcMotor.class, "lfex0");
        lrex1 = hardwareMap.get(DcMotor.class, "lrex1");
        rfex2 = hardwareMap.get(DcMotor.class, "rfex2");
        rrex3 = hardwareMap.get(DcMotor.class, "rrex3");

        Intake0ex = hardwareMap.get(DcMotorEx.class, "Intake0ex");

        Outtake1ex = hardwareMap.get(DcMotorEx.class, "Outtake1ex");
        Outtake2ex = hardwareMap.get(DcMotorEx.class, "Outtake2ex");

        Trans3ex = hardwareMap.get(DcMotorEx.class, "Trans3ex");

        z0 = hardwareMap.get(Servo.class, "z0");
        z1 = hardwareMap.get(Servo.class, "z1");

        lfex0.setDirection(DcMotorSimple.Direction.REVERSE);
        lrex1.setDirection(DcMotorSimple.Direction.FORWARD);
        rfex2.setDirection(DcMotorSimple.Direction.FORWARD);
        rrex3.setDirection(DcMotorSimple.Direction.FORWARD);

        Outtake1ex.setDirection(DcMotorSimple.Direction.REVERSE);
        Outtake2ex.setDirection(DcMotorSimple.Direction.REVERSE);

        Outtake1ex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Outtake2ex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Outtake1ex.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Outtake2ex.setMode(DcMotor.RunMode.RUN_USING_ENCODER);





    }

    public void loop() {
        strafe();
        outtake();
        intake();
        OutCam();
        telemetry.addData("Power1", Outtake1ex.getVelocity());
        telemetry.addData("Power2", Outtake2ex.getVelocity());
        telemetry.addData("z0", z0.getPosition());
        telemetry.addData("z1", z1.getPosition());
        telemetry.update();
    }

    public void strafe() {
        double yy = gamepad1.right_trigger;
        double qy = -gamepad1.left_trigger;
        double turn = gamepad1.left_stick_x / 1.5;
        double x = gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(qy) + Math.abs(yy) + Math.abs(x) + Math.abs(turn), 1);

        lfex0.setPower((qy + yy + x + turn) / denominator);
        lrex1.setPower((qy + yy - x + turn) / denominator);
        rfex2.setPower((qy + yy - x - turn) / denominator);
        rrex3.setPower((qy + yy + x - turn) / denominator);
    }

    public void outtake(){
        aprilTagWebCam.update();
        AprilTagDetection tag20 = aprilTagWebCam.getTagSpecificId(20);
        aprilTagWebCam.displayDetectionTelemetry(tag20);
        if(gamepad2.b && !tfout1){
            tfout2 = false;
            if (Outtake1ex.getPower() == 0){
                if (tag20 != null){
                    camy = tag20.ftcPose.y * 0.01;
                }
                tfout1 = true;
                tfout2 = true;
                Outtake1ex.setVelocity(PIDcontrollForOut1(ref, Outtake1ex.getVelocity()));
                Outtake2ex.setVelocity(PIDcontrollForOut2(ref, Outtake2ex.getVelocity()));
            } else if (Outtake1ex.getPower() > 0) {
                tfout1 = true;
                tfout2 = true;
                Outtake1ex.setPower(0);
                Outtake2ex.setPower(0);
            }
        } else if (!gamepad2.b && tfout2) {
            tfout1 = false;
        }
        telemetry.update();

    }

    public void intake(){
        if(gamepad1.a && !tfin1){
            tfin2 = false;
            if(Intake0ex.getPower() == 0){
                tfin1 = true;
                tfin2 = true;
                Intake0ex.setPower(6000);
                Trans3ex.setPower(-1000);

            } else if (Intake0ex.getPower() > 0) {
                tfin1 = true;
                tfin2 = true;
                Intake0ex.setPower(0);
                Trans3ex.setPower(0);
            }
        }
        else if(!gamepad1.a && tfin2){
            tfin1 = false;
        }

        if (gamepad2.a && !tftrans1){
            tftrans2 = false;
            if (Trans3ex.getPower() > 0){
                tftrans1 = true;
                tftrans2 = true;
                Trans3ex.setPower(-1000);
            } else if (Trans3ex.getPower() < 0) {
                tftrans1 = true;
                tftrans2 = true;
                Trans3ex.setPower(1000);
            }
        }
        else if (!gamepad2.a & tftrans2) {
            tftrans1 = false;
        }
    }





    //#################PID######PID#####PID########PID###################
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
    public void OutCam(){
        aprilTagWebCam.update();
        AprilTagDetection tag20 = aprilTagWebCam.getTagSpecificId(20);
        aprilTagWebCam.displayDetectionTelemetry(tag20);

        if(tag20 != null){
            double camy = tag20.ftcPose.y;
            double pos = 0.6 - camy * kp;
            pos = Range.clip(pos, 0.0, 1.0);
            z0.setPosition(pos);
            z1.setPosition(1.0 - pos);
        }
        telemetry.update();
    }

}
