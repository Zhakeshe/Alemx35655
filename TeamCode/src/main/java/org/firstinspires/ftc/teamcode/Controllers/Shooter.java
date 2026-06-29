package org.firstinspires.ftc.teamcode.Controllers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.*;

@Config
public class Shooter {
    private DcMotorEx ShooterLeft;
    private DcMotorEx ShooterRight;
    private Servo ServoLeft;
    private Servo ServoRight;
    private VoltageSensor batteryVoltageSensor;

    public static double kP = 100;
    public static double kI = 0;
    public static double kD = 20;
    public static double kF = 16;

    public static double TARGET_RPM = 1300;
    public static double RPM_CLOSE = 1300;
    public static double RPM_FAR = 1800;
    public static double MAX_RPM = 6000;

    public static double SERVO_DOWN_POS = 0.34;
    public static double SERVO_MIDDLE_POS = 0.7;
    public static double SERVO_UP_POS = 0.9;

    public enum ServosPos {
        DOWN,
        MIDDLE,
        UP
    }

    public void initialize(HardwareMap hardwareMap) {
        ShooterLeft = hardwareMap.get(DcMotorEx.class, "shooter1");
        ShooterRight = hardwareMap.get(DcMotorEx.class, "shooter2");
        ServoLeft = hardwareMap.get(Servo.class, "LeftKozel");
        ServoRight = hardwareMap.get(Servo.class, "RightKozel");

        ShooterLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        ShooterRight.setDirection(DcMotorSimple.Direction.REVERSE);

        ShooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ShooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ShooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        ShooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        ServoLeft.setDirection(Servo.Direction.FORWARD);
        ServoRight.setDirection(Servo.Direction.REVERSE);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        updatePIDF();
        setServo(ServosPos.DOWN);
    }

    private void updatePIDF() {
        double voltage = batteryVoltageSensor.getVoltage();
        PIDFCoefficients compensated = new PIDFCoefficients(
                kP,
                kI,
                kD,
                kF * 12.0 / voltage
        );
        ShooterLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, compensated);
        ShooterRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, compensated);
    }

    public void runShooter() {
        updatePIDF();
        ShooterLeft.setVelocity(TARGET_RPM);
        ShooterRight.setVelocity(TARGET_RPM);
    }

    public void stopShooter() {
        TARGET_RPM = 0;
        ShooterLeft.setPower(0);
        ShooterRight.setPower(0);
    }

    public void setTargetRPM(double rpm) {
        TARGET_RPM = clamp(rpm, 0, MAX_RPM);
    }

    public double getTargetRPM() {
        return TARGET_RPM;
    }

    public void setCloseRPM() { setTargetRPM(RPM_CLOSE); }
    public void setFarRPM() { setTargetRPM(RPM_FAR); }

    public boolean isAtSpeed(double toleranceRPM) {
        return Math.abs(ShooterLeft.getVelocity() - TARGET_RPM) < toleranceRPM
                && Math.abs(ShooterRight.getVelocity() - TARGET_RPM) < toleranceRPM;
    }

    public double getLeftRPM() { return ShooterLeft.getVelocity(); }
    public double getRightRPM() { return ShooterRight.getVelocity(); }

    public void setHoodPosition(double pos) {
        double min = Math.min(SERVO_DOWN_POS, SERVO_UP_POS);
        double max = Math.max(SERVO_DOWN_POS, SERVO_UP_POS);
        pos = clamp(pos, min, max);
        ServoLeft.setPosition(pos);
        ServoRight.setPosition(pos);
    }

    public void setServo(ServosPos pos) {
        double p = pos == ServosPos.DOWN ? SERVO_DOWN_POS :
                pos == ServosPos.MIDDLE ? SERVO_MIDDLE_POS :
                        SERVO_UP_POS;
        p = clamp(p, 0.0, 1.0);
        ServoLeft.setPosition(p);
        ServoRight.setPosition(p);
    }

    private static double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(v, max));
    }
}