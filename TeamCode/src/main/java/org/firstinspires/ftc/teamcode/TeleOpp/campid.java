package org.firstinspires.ftc.teamcode.TeleOpp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebCam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Config
@TeleOp
public class campid extends OpMode {

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

    boolean tfout1 = false;
    boolean tfout2 = true;

    boolean tfin1 = false;
    boolean tfin2 = true;

    boolean tftrans1 = false;
    boolean tftrans2 = true;

    TelemetryPacket packet = new TelemetryPacket();
    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void init() {


        Intake0ex = hardwareMap.get(DcMotorEx.class, "Intake0ex");
        Outtake1ex = hardwareMap.get(DcMotorEx.class, "Outtake1ex");
        Outtake2ex = hardwareMap.get(DcMotorEx.class, "Outtake2ex");
        Trans3ex = hardwareMap.get(DcMotorEx.class, "Trans3ex");


        z0 = hardwareMap.get(Servo.class, "z0");
        z1 = hardwareMap.get(Servo.class, "z1");

        Outtake1ex.setDirection(DcMotorSimple.Direction.REVERSE);
        Outtake2ex.setDirection(DcMotorSimple.Direction.REVERSE);

        Outtake1ex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Outtake2ex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Outtake1ex.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Outtake2ex.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        outtake();
        intake();
        telemetry.addData("Power1", Outtake1ex.getVelocity());
        telemetry.addData("Power2", Outtake2ex.getVelocity());
        packet.put("Target", ref);
        packet.put("Velocity1", Outtake1ex.getVelocity());
        packet.put("Velocity2", Outtake1ex.getVelocity());
        dashboard.sendTelemetryPacket(packet);
        telemetry.update();
    }
    public void outtake(){
        if(gamepad2.b & !tfout1){
            tfout2 = false;
            if (Outtake1ex.getPower() == 0){
                tfout1 = true;
                tfout2 = true;
                Outtake1ex.setVelocity(PIDcontrollForOut1((ref), (Outtake1ex.getVelocity())));
                Outtake2ex.setVelocity(PIDcontrollForOut2((ref), (Outtake2ex.getVelocity())));
            } else if (Outtake1ex.getPower() > 0) {
                tfout1 = true;
                tfout2 = true;
                Outtake1ex.setPower(0);
                Outtake2ex.setPower(0);
            }
        } else if (!gamepad2.b & tfout2) {
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

    public double PIDcontrollForOut1(double reference1, double state1){
        double error1 = reference1 - state1;
        integralSum1 += error1 * timer1.seconds();
        double derivative1 = (error1 - lastError1) / timer1.seconds();
        lastError1 = error1;

        timer1.reset();

        return (error1 * Kp1) + (derivative1 * Kd1) + (integralSum1 * Ki1) + (reference1 * Kf1);


    }
    public double PIDcontrollForOut2(double reference2, double state2){
        double error2 = reference2 - state2;
        integralSum2 += error2 * timer2.seconds();
        double derivative2 = (error2 - lastError2) / timer2.seconds();
        lastError2 = error2;

        timer2.reset();

        return (error2 * Kp2) + (derivative2 * Kd2) + (integralSum2 * Ki2) + (reference2 * Kf2);


    }
}
