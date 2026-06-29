package org.firstinspires.ftc.teamcode.Controllers;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Action {
    private Intake intake;
    private Shooter shooter;
    private Turret turret;

    public static double TARGET_X = 144;
    public static double TARGET_Y = 144;
    public static double TARGET_BLUE_X = 0;
    public static double TARGET_BLUE_Y = 144;

    public static double RPM_CLOSE = 1250;
    public static double RPM_FAR = 2000;

    private boolean shooterEnabled = false;
    private boolean autoModeActive = false;
    public static boolean isBlue = false;

    public Action(HardwareMap hardwareMap) {
        intake = new Intake();
        shooter = new Shooter();
        turret = new Turret();

        intake.initialize(hardwareMap);
        shooter.initialize(hardwareMap);
        turret.init(hardwareMap);
    }

    public void updateAll(Pose robotPose) {

        if (shooterEnabled) {
            shooter.runShooter();
        } else {
            shooter.stopShooter();
        }

        turret.update(robotPose);
    }

    public void Sequence() {
        intake.setServo(Intake.IntakeState.CLOSED);
        intake.setIntakePower(1);
    }

    public void FarSequence() {
        intake.setServo(Intake.IntakeState.CLOSED);
        intake.setIntakePower(0.75);
    }

    public void take() {
        intake.setServo(Intake.IntakeState.OPEN);
        intake.setIntakePower(1);
    }

    public void stopTake() {
        intake.setIntakePower(0);
    }

    public void reverseIntake() {
        intake.setIntakePower(-1);
    }

    public static Pose getActiveTarget() {
        return isBlue ? new Pose(TARGET_BLUE_X, TARGET_BLUE_Y) : new Pose(TARGET_X, TARGET_Y);
    }

    public void startAutoShooter() {
        this.autoModeActive = true;
        this.shooterEnabled = true;
    }

    public double getShooterTargetRPM() {
        return shooter.getTargetRPM();
    }

    public void runCloseShooter(){
        this.shooterEnabled = true;
        shooter.setTargetRPM(1200);
    }

    public void runFarShoot(){
        this.shooterEnabled = true;
        shooter.setTargetRPM(1800);
    }

    public void triggerShooter() {
        shooterEnabled = true;
        if (!autoModeActive) {
            shooter.setTargetRPM(RPM_CLOSE);
        }
    }

    public void stopShooter() {
        shooterEnabled = false;
        shooter.stopShooter();
    }

    public double getShooterCurrentRPML() {
        return shooter.getLeftRPM();
    }

    public double getShooterCurrentRPMR() {
        return shooter.getRightRPM();
    }

    public void setTurretMode(Turret.Mode mode) {
        turret.setMode(mode);
    }

    public double getTurretAngle() {
        return turret.getTargetAngle();
    }

    public void toggleAutoMode() {
        autoModeActive = !autoModeActive;
    }

    public boolean isAutoModeActive() {
        return autoModeActive;
    }

    public void runFarShooter() {
        shooter.setTargetRPM(RPM_FAR);
    }

    public void runShooterRaw() {
        shooterEnabled = true;
    }

    public void servoUp() {
        shooter.setServo(Shooter.ServosPos.UP);
    }

    public void servoMidle() {
        shooter.setServo(Shooter.ServosPos.MIDDLE);
    }

    public void servoDown() {
        shooter.setServo(Shooter.ServosPos.DOWN);
    }
}