package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
@Config
public class pidConfig extends OpMode {
    double integralSum1 = 0;

    public static double Kp1 = 0.007;
    public static double Ki1 = 0;
    public static double Kd1 = 0.0015;

    public static double Kf1 = 0.97;
    private static final double MIN_PID_DT = 1e-3;
    ElapsedTime timer1 = new ElapsedTime();
    private double lastError1 = 0;

    public static double ref = 1200;

    boolean tf = false;
    boolean tf2 = false;

    private DcMotorEx shooter, intake, transfer;

    private Servo stopper;

    TelemetryPacket packet = new TelemetryPacket();
    FtcDashboard dashboard = FtcDashboard.getInstance();

    GoBildaPinpointDriver pinpointDriver;

    @Override
    public void init() {
        stopper = hardwareMap.get(Servo.class, "stopper");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter1");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");

        shooter.setDirection(DcMotorSimple.Direction.FORWARD);

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        pinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class, Constants.PINPOINT_HARDWARE_NAME);
        pinpointDriver.setOffsets(-15,-7.5, DistanceUnit.CM);

        pinpointDriver.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        pinpointDriver.resetPosAndIMU();
    }

    @Override
    public void loop() {
        pinpointDriver.update();

        Pose2D pos = pinpointDriver.getPosition();

        double x = pos.getX(DistanceUnit.CM);
        double y = pos.getY(DistanceUnit.CM);

        double ds = Math.sqrt(x * x + y * y);

        telemetry.addData("X: ", x);
        telemetry.addData("Y: ", y);
        telemetry.addData("xy: ", ds);
        telemetry.addData("ref: ", ref);

        packet.put("Target", ref);
        packet.put("Velocity1", shooter.getVelocity());
        telemetry.addData("v1: ", shooter.getVelocity());

        dashboard.sendTelemetryPacket(packet);
        telemetry.update();
        if (gamepad1.y){
            stopper.setPosition(0.05);
        }else {
            stopper.setPosition(0.4);
        }

        if (gamepad1.a && !tf){
            tf2 = false;
            if (shooter.getPower() == 0){
                tf = true;
                tf2 = true;
                shoot(true);
            }else if(shooter.getPower() > 0){
                tf = true;
                tf2 = true;
                shoot(false);
            }
        } else if (!gamepad1.a && tf2) {
            tf = false;
        }

        if (gamepad1.b){
            pinpointDriver.setPosition(new Pose2D(DistanceUnit.CM, 0, 0, AngleUnit.DEGREES, 0));
            pinpointDriver.update();
        }
    }

    public void shoot(boolean tf){
        if (tf){
            shooter.setVelocity(PIDcontrollHigh1(ref, shooter.getVelocity()));
            intake.setPower(1);
            transfer.setPower(1);
        }else {
            shooter.setPower(0);
            intake.setPower(0);
            transfer.setPower(0);
        }
    }

    public double getRef(double dis){

        return ((0.000304493 * dis * dis * dis - 0.150331 * dis * dis + 24.36157 * dis + 0));
    }

    public double PIDcontrollHigh1(double reference1, double state1){
        double error1 = reference1 - state1;
        double dt = Math.max(timer1.seconds(), MIN_PID_DT);
        integralSum1 += error1 * dt;
        double derivative1 = (error1 - lastError1) / dt;
        lastError1 = error1;

        timer1.reset();

        return (error1 * Kp1) + (derivative1 * Kd1) + (integralSum1 * Ki1) + (reference1 * Kf1);

    }
}
