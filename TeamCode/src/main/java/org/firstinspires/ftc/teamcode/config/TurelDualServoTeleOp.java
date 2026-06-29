package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Config
@TeleOp(name = "Turel Dual Servo", group = "Digital")
public class TurelDualServoTeleOp extends OpMode {

    public static double CENTER_POSITION       = 0.50;
    public static double MIN_POSITION          = 0.10;
    public static double MAX_POSITION          = 0.90;
    public static double POSITION_PER_DEGREE   = 1.0 / 300.0;
    public static double CENTER_REL_ANGLE_DEG  = 0.0;
    public static double TURRET_DIRECTION      = -1.0;
    public static boolean MIRROR_SECOND_SERVO  = false;

    public static double START_X               = 108.0;
    public static double START_Y               = 58.0;
    public static double START_HEADING_DEG     = 0.0;
    public static double FIELD_LOCK_GAIN       = 1.0;
    public static double TURN_RATE_COMP_GAIN   = 1.46;
    public static double TURN_INPUT_COMP_GAIN  = 1.18;
    public static double HEADING_LOOKAHEAD_SEC = 0.25;
    public static double TURN_INPUT_FF_DEG     = 18.0;
    public static double LARGE_ERROR_START_DEG = 12.0;
    public static double LARGE_ERROR_GAIN      = 0.55;
    public static double TURN_DEADBAND         = 0.05;
    public static double TURN_SETTLED_DEG_S    = 8.0;
    public static double AIM_DEADBAND_DEG      = 0.02;

    public static double COARSE_STEP_DEG      = 10.0;
    public static double FINE_STEP_DEG        = 2.0;
    public static double STICK_SPEED_DEG_S    = 120.0;
    public static double STICK_DEADZONE       = 0.08;

    public static double MAX_SLEW_PER_SEC     = 1.5;

    private Follower follower;
    private Servo    turel;
    private Servo    turel1;

    private double lockedFieldHeadingDeg;
    private double targetPosition;
    private double appliedPosition;
    private double lastLoopTime;
    private double lastHeadingDeg;
    private double continuousHeadingDeg;
    private double turnRateDegPerSec;

    private boolean dpadLeftWas, dpadRightWas, dpadUpWas, dpadDownWas;
    private boolean aWas, yWas;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(new Pose(START_X, START_Y, Math.toRadians(START_HEADING_DEG)));

        turel  = hardwareMap.get(Servo.class, "turel");
        turel1 = hardwareMap.get(Servo.class, "turel1");

        lockedFieldHeadingDeg = START_HEADING_DEG;
        lastHeadingDeg        = START_HEADING_DEG;
        continuousHeadingDeg  = START_HEADING_DEG;
        turnRateDegPerSec     = 0.0;
        targetPosition        = CENTER_POSITION;
        appliedPosition       = CENTER_POSITION;
        lastLoopTime          = 0.0;

        applyPosition(CENTER_POSITION);
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        follower.update();
        lastLoopTime = getRuntime();
    }

    @Override
    public void loop() {
        double now = getRuntime();

        double dt = Math.max(0.001, now - lastLoopTime);
        lastLoopTime = now;

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true
        );
        follower.update();

        updateHeadingState(dt);

        handleTurretButtons(dt);

        targetPosition = computeFieldLockPosition();

        appliedPosition = moveTowards(appliedPosition, targetPosition, MAX_SLEW_PER_SEC * dt);
        applyPosition(appliedPosition);

        Pose pose = follower.getPose();
        telemetry.addData("Robot heading deg", "%.2f", Math.toDegrees(pose.getHeading()));
        telemetry.addData("Locked field heading", "%.2f", lockedFieldHeadingDeg);
        telemetry.addData("Relative turret deg", "%.2f", getRelativeTurretDeg());
        telemetry.addData("Target pos", "%.4f", targetPosition);
        telemetry.addData("Applied pos", "%.4f", appliedPosition);
        telemetry.addData("Turn rate deg/s", "%.1f", turnRateDegPerSec);
        telemetry.addLine("gamepad2: stick=adjust  dpad L/R=coarse  U/D=fine  A=center  Y=reset pose");
        telemetry.update();
    }

    @Override
    public void stop() {
        applyPosition(CENTER_POSITION);
    }

    private void handleTurretButtons(double dt) {
        boolean dpadLeft  = gamepad2.dpad_left;
        boolean dpadRight = gamepad2.dpad_right;
        boolean dpadUp    = gamepad2.dpad_up;
        boolean dpadDown  = gamepad2.dpad_down;
        boolean a         = gamepad2.a;
        boolean y         = gamepad2.y;

        if (dpadLeft  && !dpadLeftWas)  lockedFieldHeadingDeg -= COARSE_STEP_DEG;
        if (dpadRight && !dpadRightWas) lockedFieldHeadingDeg += COARSE_STEP_DEG;
        if (dpadDown  && !dpadDownWas)  lockedFieldHeadingDeg -= FINE_STEP_DEG;
        if (dpadUp    && !dpadUpWas)    lockedFieldHeadingDeg += FINE_STEP_DEG;

        if (a && !aWas) {
            lockedFieldHeadingDeg = continuousHeadingDeg + CENTER_REL_ANGLE_DEG;
        }

        if (y && !yWas) {
            follower.setPose(new Pose(START_X, START_Y, Math.toRadians(START_HEADING_DEG)));
            continuousHeadingDeg = START_HEADING_DEG;
            lastHeadingDeg       = START_HEADING_DEG;
        }

        double stick = gamepad2.left_stick_x;
        if (Math.abs(stick) >= STICK_DEADZONE) {
            lockedFieldHeadingDeg += stick * STICK_SPEED_DEG_S * dt;
        }

        dpadLeftWas  = dpadLeft;
        dpadRightWas = dpadRight;
        dpadUpWas    = dpadUp;
        dpadDownWas  = dpadDown;
        aWas         = a;
        yWas         = y;
    }

    private double computeFieldLockPosition() {

        double baseRelDeg   = normDeg(lockedFieldHeadingDeg - continuousHeadingDeg) * FIELD_LOCK_GAIN;
        double boostedRelDeg = applyLargeErrorBoost(baseRelDeg);

        double turnInput = gamepad1.right_stick_x;
        if (Math.abs(turnInput) < TURN_DEADBAND) turnInput = 0.0;

        double effectiveTurnRate = turnRateDegPerSec;
        if (turnInput == 0.0 && Math.abs(effectiveTurnRate) < TURN_SETTLED_DEG_S) {
            effectiveTurnRate = 0.0;
        }

        double turnRateComp  = effectiveTurnRate * HEADING_LOOKAHEAD_SEC * TURN_RATE_COMP_GAIN;
        double turnInputComp = turnInput * TURN_INPUT_FF_DEG * TURN_INPUT_COMP_GAIN;

        double relDeg = clampRelDeg(boostedRelDeg - turnRateComp + turnInputComp);

        if (Math.abs(baseRelDeg) < AIM_DEADBAND_DEG
                && Math.abs(turnRateComp) < AIM_DEADBAND_DEG
                && Math.abs(turnInputComp) < AIM_DEADBAND_DEG) {
            relDeg = 0.0;
        }

        return relDegToPos(relDeg);
    }

    private void updateHeadingState(double dt) {
        double currentDeg = Math.toDegrees(follower.getPose().getHeading());
        double delta      = normDeg(currentDeg - lastHeadingDeg);
        continuousHeadingDeg += delta;
        lastHeadingDeg        = currentDeg;
        if (dt <= 0.0) return;
        double measured = delta / dt;

        turnRateDegPerSec = 0.70 * turnRateDegPerSec + 0.30 * measured;
    }

    private double getRelativeTurretDeg() {
        return clampRelDeg(normDeg(lockedFieldHeadingDeg - continuousHeadingDeg));
    }

    private double applyLargeErrorBoost(double relDeg) {
        double abs = Math.abs(relDeg);
        if (abs <= LARGE_ERROR_START_DEG) return relDeg;
        return Math.signum(relDeg) * (abs + (abs - LARGE_ERROR_START_DEG) * LARGE_ERROR_GAIN);
    }

    private double clampRelDeg(double relDeg) {
        double a = posToRelDeg(MIN_POSITION);
        double b = posToRelDeg(MAX_POSITION);
        return Range.clip(relDeg, Math.min(a, b), Math.max(a, b));
    }

    private double relDegToPos(double relDeg) {
        double raw = CENTER_POSITION + (relDeg - CENTER_REL_ANGLE_DEG) * POSITION_PER_DEGREE * TURRET_DIRECTION;
        return Range.clip(raw, MIN_POSITION, MAX_POSITION);
    }

    private double posToRelDeg(double position) {
        return ((position - CENTER_POSITION) / POSITION_PER_DEGREE) * TURRET_DIRECTION + CENTER_REL_ANGLE_DEG;
    }

    private double moveTowards(double current, double target, double maxDelta) {
        double diff = target - current;
        if (Math.abs(diff) <= maxDelta) return target;
        return current + Math.signum(diff) * maxDelta;
    }

    private double normDeg(double deg) {
        while (deg > 180.0)  deg -= 360.0;
        while (deg < -180.0) deg += 360.0;
        return deg;
    }

    private void applyPosition(double position) {
        position = Range.clip(position, MIN_POSITION, MAX_POSITION);
        turel.setPosition(position);
        turel1.setPosition(MIRROR_SECOND_SERVO ? 1.0 - position : position);
    }
}
