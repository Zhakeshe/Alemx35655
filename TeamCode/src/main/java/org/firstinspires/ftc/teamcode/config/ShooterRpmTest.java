package org.firstinspires.ftc.teamcode.config;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
@TeleOp(name = "Shooter RPM Test", group = "Digital")
public class ShooterRpmTest extends OpMode {

    public static double TARGET_RPM = 170.0;
    public static double RPM_TOLERANCE = 25.0;

    public static double STOPPER_OPEN = 0.05;
    public static double STOPPER_CLOSED = 0.45;

    public static double SHOOTER_POWER_LIMIT = 1.0;

    private DcMotorEx shooter1;
    private DcMotorEx turel;
    private Servo stopper;

    private double shooterTicksPerRev = 1.0;
    private double targetVelocityTicksPerSec = 0.0;

    private boolean enabled = false;

    @Override
    public void init() {
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        turel = hardwareMap.get(DcMotorEx.class, "turel");
        stopper = hardwareMap.get(Servo.class, "stopper");

        shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
        turel.setDirection(DcMotorSimple.Direction.FORWARD);

        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        turel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooterTicksPerRev = shooter1.getMotorType().getTicksPerRev();
        updateTargetVelocity();

        closeStopper();
        stopShooter();
    }

    @Override
    public void start() {
        enabled = true;
        updateTargetVelocity();
        closeStopper();
    }

    @Override
    public void loop() {
        updateTargetVelocity();

        if (enabled) {
            applyShooterVelocity();
        } else {
            stopShooter();
        }

        boolean atSpeed = shooterAtTargetRpm();

        if (enabled && atSpeed) {
            openStopper();
        } else {
            closeStopper();
        }

        if (gamepad1.a) {
            enabled = true;
        }
        if (gamepad1.b) {
            enabled = false;
            stopShooter();
            closeStopper();
        }

        telemetry.addData("Enabled", enabled);
        telemetry.addData("Target RPM", TARGET_RPM);
        telemetry.addData("Tolerance", RPM_TOLERANCE);
        telemetry.addData("Shooter RPM", ticksPerSecondToRpm(shooter1.getVelocity(), shooterTicksPerRev));
        telemetry.addData("Turel RPM", ticksPerSecondToRpm(turel.getVelocity(), shooterTicksPerRev));
        telemetry.addData("At Speed", atSpeed);
        telemetry.addData("Stopper", stopper.getPosition());
        telemetry.addLine("A = enable, B = stop");
        telemetry.update();
    }

    private void updateTargetVelocity() {
        targetVelocityTicksPerSec = rpmToTicksPerSecond(TARGET_RPM, shooterTicksPerRev);
    }

    private void applyShooterVelocity() {
        shooter1.setVelocity(targetVelocityTicksPerSec);
        turel.setVelocity(targetVelocityTicksPerSec);
    }

    private void stopShooter() {
        shooter1.setVelocity(0);
        turel.setVelocity(0);
    }

    private boolean shooterAtTargetRpm() {
        double shooterRpm = ticksPerSecondToRpm(shooter1.getVelocity(), shooterTicksPerRev);
        double turelRpm = ticksPerSecondToRpm(turel.getVelocity(), shooterTicksPerRev);

        return Math.abs(shooterRpm - TARGET_RPM) <= RPM_TOLERANCE
                && Math.abs(turelRpm - TARGET_RPM) <= RPM_TOLERANCE;
    }

    private double rpmToTicksPerSecond(double rpm, double ticksPerRev) {
        return rpm * ticksPerRev / 60.0;
    }

    private double ticksPerSecondToRpm(double ticksPerSecond, double ticksPerRev) {
        return ticksPerSecond * 60.0 / ticksPerRev;
    }

    private void openStopper() {
        stopper.setPosition(STOPPER_OPEN);
    }

    private void closeStopper() {
        stopper.setPosition(STOPPER_CLOSED);
    }
}
