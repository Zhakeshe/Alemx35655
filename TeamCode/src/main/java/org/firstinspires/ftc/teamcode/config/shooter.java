package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Config
public class shooter {

    //####################################
    double integralSum1 = 0;

    public static double Kp1 = 0.007;
    public static double Ki1 = 0;
    public static double Kd1 = 0.0015;

    public static double Kf1 = 0.97;
    ElapsedTime timer1 = new ElapsedTime();
    private double lastError1 = 0;

    public static double ref;


    //######################################

    private DcMotorEx shooter1, intake, transfer;

    private Servo stopper;

    boolean tfsh = false;
    boolean tfsh1 = false;
    boolean tf = false;

    GoBildaPinpointDriver pinpointDriver;


    public void init(HardwareMap hwMap){
        shooter1 = hwMap.get(DcMotorEx.class, "shooter1");
        intake = hwMap.get(DcMotorEx.class, "intake");
        transfer = hwMap.get(DcMotorEx.class, "transfer");
        stopper = hwMap.get(Servo.class, "stopper");

        shooter1.setDirection(DcMotorSimple.Direction.FORWARD);

        shooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pinpointDriver = hwMap.get(GoBildaPinpointDriver.class, "pinpoint1");
        pinpointDriver.setOffsets(-15,-7.5, DistanceUnit.CM);


        pinpointDriver.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        pinpointDriver.resetPosAndIMU();

    }

    public void shooteryep(boolean rb, boolean dup, boolean dd, boolean g1y, Telemetry telemetry){

        shoot(tf);

        pinpointDriver.update();

        Pose2D pos = pinpointDriver.getPosition();

        double x = pos.getX(DistanceUnit.CM);
        double y = pos.getY(DistanceUnit.CM);

        double ds = Math.sqrt(x * x + y * y);

        telemetry.addData("x: ", x);
        telemetry.addData("y: ", y);
        telemetry.addData("xy: ", ds);
        telemetry.addData("ref: ", ref);
        telemetry.addData("shooter: : ", shooter1.getVelocity());
        telemetry.update();
        ref = getRef(ds);

        if (rb && !tfsh){
            tfsh1 = false;
            if (shooter1.getPower() == 0){
                tfsh = true;
                tfsh1 = true;
                tf = true;
            } else if (shooter1.getPower() > 0) {
                tfsh = true;
                tfsh1 = true;
                tf = false;
            }
        } else if (!rb && tfsh1) {
            tfsh = false;
        }


        if (dup){
            pinpointDriver.setPosition(new Pose2D(DistanceUnit.CM, 0, 0, AngleUnit.DEGREES, 0));
            pinpointDriver.update();
        }else if(dd){
            pinpointDriver.setPosition(new Pose2D(DistanceUnit.CM, 50.4901, -300, AngleUnit.DEGREES, 0));
            pinpointDriver.update();
        }


        if (g1y){
            stopper.setPosition(0.05);
        }else {
            stopper.setPosition(0.4);
        }
    }

    public void shoot(boolean tf){
        if (tf){
            shooter1.setVelocity(PIDcontrollHigh1(ref, shooter1.getVelocity()));
            intake.setPower(1);
            transfer.setPower(1);
        }else {
            shooter1.setPower(0);
            intake.setPower(0);
            transfer.setPower(0);
        }
    }

    public double getRef(double dis){
        //y=0.000608514x^{3}-0.25286x^{2}+34.995x-353.11841
        return (Range.clip(0.000608514 * dis * dis * dis - 0.25286 * dis * dis + 34.995 * dis - 353.11841, 0, 1720));
        //############################################################################################################################
        //y=-0.0000024136x^{4}+0.00181906x^{3}-0.475003x^{2}+52.68363x-869.18476
        //return (Range.clip(0.0000024136 * dis * dis * dis * dis + 0.00181906 * dis * dis * dis + 0.475003 * dis * dis + 52.68363 * dis - 869.18476, 0, 1725));
    }

    public double PIDcontrollHigh1(double reference1, double state1){
        double error1 = reference1 - state1;
        integralSum1 += error1 * timer1.seconds();
        double derivative1 = (error1 - lastError1) / timer1.seconds();
        lastError1 = error1;

        timer1.reset();

        return (error1 * Kp1) + (derivative1 * Kd1) + (integralSum1 * Ki1) + (reference1 * Kf1);


    }
}
