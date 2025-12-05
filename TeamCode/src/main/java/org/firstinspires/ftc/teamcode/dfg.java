package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.*;
import java.util.List;

@Config
public class dfg {

    private DcMotorEx ShooterLeft;
    private DcMotorEx ShooterRight;
    private Servo ServoLeft;
    private Servo ServoRight;

    private VoltageSensor batteryVoltageSensor;

    public static double kP = 30;
    public static double kI = 0;
    public static double kD = 14;
    public static double kF = 10;

    private PIDFCoefficients pidfShooterCoefficients =
            new PIDFCoefficients(kP, kI, kD, kF);

    public static double RPM_BELOW_13 = 1350;
    public static double RPM_13_TO_13_2 = 1315;
    public static double RPM_13_2_TO_13_5 = 1300;
    public static double RPM_13_5_TO_14 = 1280;

    public enum ServosPos {
        DIRECTION_DOWN(0.8),
        DIRECTION_UP(0.4);

        private final double position;
        ServosPos(double pos) { this.position = pos; }
        public double getPos() { return position; }
    }

    public void initialize(HardwareMap hardwareMap) {

        ShooterLeft = hardwareMap.get(DcMotorEx.class, "ShooterLeft");
        ShooterRight = hardwareMap.get(DcMotorEx.class, "ShooterRight");

        ServoLeft = hardwareMap.get(Servo.class, "LeftKozel");
        ServoRight = hardwareMap.get(Servo.class, "RightKozel");

        ShooterLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        ShooterRight.setDirection(DcMotorSimple.Direction.FORWARD);

        ShooterLeft.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER, pidfShooterCoefficients);
        ShooterRight.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER, pidfShooterCoefficients);

        ServoRight.setDirection(Servo.Direction.REVERSE);
        ServoLeft.setDirection(Servo.Direction.FORWARD);


        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
    }



    public double getBatteryVoltage() {
        return batteryVoltageSensor.getVoltage();
    }

    public double getCompensatedRPM() {
        double v = getBatteryVoltage();

        if (v < 13.0) return RPM_BELOW_13;
        if (v < 13.2) return RPM_13_TO_13_2;
        if (v < 13.5) return RPM_13_2_TO_13_5;
        return RPM_13_5_TO_14;
    }

    public void runShooterVoltageCompensated() {
        double targetRPM = getCompensatedRPM();
        setVelocity(targetRPM);
    }


    public void setVelocity(double velocity) {
        ShooterLeft.setVelocity(velocity);
        ShooterRight.setVelocity(velocity);
    }

    public void SetShooterPower(double power) {
        ShooterLeft.setPower(power);
        ShooterRight.setPower(power);
    }

    public void setServoPosition(double pos) {
        ServoLeft.setPosition(pos);
        ServoRight.setPosition(pos);
    }


    public double getRightVelocity() {
        return ShooterRight.getVelocity();
    }

    public boolean CheckVelocity(double targetVelocity, double offset) {
        return Math.abs(ShooterLeft.getVelocity())
                >= Math.abs(targetVelocity - offset);
    }
}