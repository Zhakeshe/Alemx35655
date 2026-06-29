package org.firstinspires.ftc.teamcode.config;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.List;

@Configurable
@Autonomous(name = "ZHORIK AUTO 3", group = "Digital")
public class AutoRed3 extends OpMode {

    public static double SHOOTER_VELO_MAX = 1200.0;
    public static double SHOOTER_VELO_IDLE = 600.0;

    public static double INTAKE_POWER_COLLECT = 1.0;
    public static double INTAKE_POWER_HOLD = 1.0;

    public static double TRANSFER_POWER_HOLD = 1.0;

    public static double INTAKE_POWER_SHOOT = 1.0;
    public static double TRANSFER_POWER_SHOOT = 1.0;

    public static double STOPPER_OPEN = 0.07;
    public static double STOPPER_CLOSED = 0.48;
    public static double SHOOTER_READY_TOLERANCE = 30.0;

    public static double FIRST_SHOT_WARMUP_DELAY = 0.30;
    public static double FOLLOWUP_SHOT_WARMUP_DELAY = 0.12;

    public static double SHOT_FORCE_OPEN_DELAY = 0.80;

    public static double TIME_TO_SHOOT = 0.50;
    public static double TUREL_SHOT_1_POSITION = 0.5;
    public static double TUREL_SHOT_2_POSITION = 0.5;
    public static double TUREL_PARK_POSITION = 0.5;

    public static double TIME_TO_COLLECT_WAIT = 1.1;

    public static double FIRST_SHOOT_DELAY = 1.23;
    public static double EMERGENCY_EXIT_TIME = 29.0;
    public static double MAX_PATH_TIMEOUT = 3.5;

    private Follower follower;
    private DcMotorEx intake;
    private DcMotorEx shooter1;
    private DcMotorEx shooter2;
    private DcMotorEx transfer;
    private Servo turel;
    private Servo turel1;
    private Servo stopper;
    private double turelHoldPosition;
    private double turel1HoldPosition;

    private final List<Double> veloLog1 = new ArrayList<>();
    private final List<Double> veloLog2 = new ArrayList<>();
    private final List<Double> timeLog = new ArrayList<>();
    private boolean logging = true;

    private enum ShootSubState {
        WARMUP,
        FIRING
    }
    private ShootSubState shootSubState = ShootSubState.WARMUP;

    private enum PathState {
        START_TO_SHOOT,
        SHOOT_1,
        GO_COLLECT_1,
        RETURN_1,
        SHOOT_2,
        GO_COLLECT_2,
        RETURN_2,
        SHOOT_3,
        GO_COLLECT_3,
        WAIT_COLLECT_3,
        GO_COLLECT_35,
        RETURN_3,
        SHOOT_4,
        GO_COLLECT_4,
        WAIT_COLLECT_4,
        GO_COLLECT_45,
        RETURN_4,
        SHOOT_5,
        PARK,
        END
    }

    private PathState pathState = PathState.START_TO_SHOOT;
    private final Timer stateTimer = new Timer();
    private boolean firstEnter = true;
    private boolean emergencyExit = false;

    private PathChain startToShoot;
    private PathChain goCollect1;
    private PathChain return1;
    private PathChain goCollect2;
    private PathChain return2;
    private PathChain goCollect3;
    private PathChain return3;
    private PathChain goCollect4;
    private PathChain return4;
    private PathChain goCollect35;
    private PathChain goCollect45;
    private PathChain park;

    private final Pose startPose     = new Pose(105, 134, Math.toRadians(0));
    private final Pose shootPose     = new Pose(93.2111801242236, 83.95496894409939, Math.toRadians(0));

    private final Pose collectPose1        = new Pose(124, 62, Math.toRadians(0));
    private final Pose collectControlPose1 = new Pose(84.59549689440996, 62, Math.toRadians(0));
    private final Pose returnControlPose1  = new Pose(105.24245016260936, 71.7388417843294, Math.toRadians(0));

    private final Pose collectPose2 = new Pose(119, 91, Math.toRadians(0));

    private final Pose gatePose        = new Pose(127, 66, Math.toRadians(-5));
    private final Pose gateDeepPose    = new Pose(130.22558355503236, 56.46836679847239, Math.toRadians(0));
    private final Pose gateDeepControl = new Pose(124.81271413776464, 56.211667871285876, Math.toRadians(0));

    private final Pose parkPose = new Pose(108, 58, Math.toRadians(0));

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);

        intake   = hardwareMap.get(DcMotorEx.class, "intake");
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        turel    = hardwareMap.get(Servo.class, "turel");
        turel1   = hardwareMap.get(Servo.class, "turel1");
        stopper  = hardwareMap.get(Servo.class, "stopper");

        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        transfer.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        setTurelShot1Position();
        holdTurelServos();

        buildPaths();
        stopAllMechanisms();
        closeStopper();
    }

    @Override
    public void start() {
        pathState    = PathState.START_TO_SHOOT;
        firstEnter   = true;
        emergencyExit = false;
        logging      = true;
        veloLog1.clear();
        veloLog2.clear();
        timeLog.clear();

        stateTimer.resetTimer();
        closeStopper();
        setTurelShot1Position();
        holdTurelServos();

        shooter1.setVelocity(SHOOTER_VELO_MAX);
        shooter2.setVelocity(SHOOTER_VELO_MAX);

        intake.setPower(INTAKE_POWER_HOLD);
        transfer.setPower(TRANSFER_POWER_HOLD);
    }

    @Override
    public void loop() {
        follower.update();
        holdTurelServos();

        if (logging && getRuntime() < 30.0) {
            timeLog.add(getRuntime());
            veloLog1.add(shooter1.getVelocity());
            veloLog2.add(shooter2.getVelocity());
        }

        if (pathState == PathState.END) {
            stopAllMechanisms();
            holdTurelServos();
            closeStopper();

            if (logging) {
                logging = false;
                telemetry.addLine("=== VELOCITY LOG ===");
                for (int i = 0; i < timeLog.size(); i++) {
                    telemetry.addData(
                        String.format("t=%.2f", timeLog.get(i)),
                        String.format("v1=%.0f  v2=%.0f", veloLog1.get(i), veloLog2.get(i))
                    );
                }
            }
            telemetry.addData("State", "END / IDLE");
            telemetry.update();
            return;
        }

        updateFSM();

        telemetry.addData("State", pathState);
        telemetry.addData("Shoot SubState", shootSubState);
        telemetry.addData("Shooter1 Velo", "%.1f", shooter1.getVelocity());
        telemetry.addData("Shooter2 Velo", "%.1f", shooter2.getVelocity());
        telemetry.addData("Velo Diff", "%.1f", SHOOTER_VELO_MAX - shooter1.getVelocity());
        telemetry.addData("ShooterReady", shooterReady());
        telemetry.addData("Pose", follower.getPose());
        telemetry.update();
    }

    private void updateFSM() {
        if (!emergencyExit && getRuntime() > EMERGENCY_EXIT_TIME) {
            emergencyExit = true;
            pathState = PathState.PARK;
            firstEnter = true;
        }

        if (pathState != PathState.PARK && pathState != PathState.END) {
            shooter1.setVelocity(SHOOTER_VELO_MAX);
            shooter2.setVelocity(SHOOTER_VELO_MAX);
            updateTurelForCurrentState();
        }

        switch (pathState) {
            case START_TO_SHOOT:
                followOnce(startToShoot);

                keepIntakeRunning();
                if (pathTimedOut()) {
                    emergencyPark();
                } else if (stateTimer.getElapsedTimeSeconds() > FIRST_SHOOT_DELAY) {
                    next(PathState.SHOOT_1);
                }
                break;

            case SHOOT_1:
                shootFor(PathState.GO_COLLECT_1);
                break;

            case GO_COLLECT_1:
                followOnce(goCollect1);
                collectIntake();
                advanceWhenPathDone(PathState.RETURN_1);
                break;

            case RETURN_1:
                followOnce(return1);
                keepIntakeRunning();
                advanceWhenPathDone(PathState.SHOOT_2);
                break;

            case SHOOT_2:
                shootFor(PathState.GO_COLLECT_2);
                break;

            case GO_COLLECT_2:
                followOnce(goCollect2);
                collectIntake();
                advanceWhenPathDone(PathState.RETURN_2);
                break;

            case RETURN_2:
                followOnce(return2);
                keepIntakeRunning();
                advanceWhenPathDone(PathState.SHOOT_3);
                break;

            case SHOOT_3:
                shootFor(PathState.GO_COLLECT_3);
                break;

            case GO_COLLECT_3:
                followOnce(goCollect3);
                collectIntake();
                advanceWhenPathDone(PathState.WAIT_COLLECT_3);
                break;

            case WAIT_COLLECT_3:
                collectIntake();
                if (stateTimer.getElapsedTimeSeconds() > TIME_TO_COLLECT_WAIT) {
                    next(PathState.GO_COLLECT_35);
                }
                break;

            case GO_COLLECT_35:
                followOnce(goCollect35);
                collectIntake();
                advanceWhenPathDone(PathState.RETURN_3);
                break;

            case RETURN_3:
                followOnce(return3);
                keepIntakeRunning();
                advanceWhenPathDone(PathState.SHOOT_4);
                break;

            case SHOOT_4:
                shootFor(PathState.GO_COLLECT_4);
                break;

            case GO_COLLECT_4:
                followOnce(goCollect4);
                collectIntake();
                advanceWhenPathDone(PathState.WAIT_COLLECT_4);
                break;

            case WAIT_COLLECT_4:
                collectIntake();
                if (stateTimer.getElapsedTimeSeconds() > TIME_TO_COLLECT_WAIT) {
                    next(PathState.GO_COLLECT_45);
                }
                break;

            case GO_COLLECT_45:
                followOnce(goCollect45);
                collectIntake();
                advanceWhenPathDone(PathState.RETURN_4);
                break;

            case RETURN_4:
                followOnce(return4);
                keepIntakeRunning();
                advanceWhenPathDone(PathState.SHOOT_5);
                break;

            case SHOOT_5:
                shootFor(PathState.PARK);
                break;

            case PARK:
                followOnce(park);
                setTurelParkPosition();
                holdTurelServos();
                shooter1.setVelocity(SHOOTER_VELO_IDLE);
                shooter2.setVelocity(SHOOTER_VELO_IDLE);
                intake.setPower(0);
                transfer.setPower(0);
                closeStopper();
                if (parkDone()) next(PathState.END);
                break;

            case END:
                stopAllMechanisms();
                setTurelParkPosition();
                holdTurelServos();
                closeStopper();
                break;
        }
    }

    private void shootFor(PathState nextState) {
        if (firstEnter) {
            closeStopper();
            shootSubState = ShootSubState.WARMUP;
            stateTimer.resetTimer();
            firstEnter = false;
        }

        switch (shootSubState) {
            case WARMUP:
                closeStopper();

                keepIntakeRunning();

                if (canOpenStopper(stateTimer.getElapsedTimeSeconds())) {
                    openStopper();
                    intake.setPower(INTAKE_POWER_SHOOT);
                    transfer.setPower(TRANSFER_POWER_SHOOT);

                    shootSubState = ShootSubState.FIRING;
                    stateTimer.resetTimer();
                }
                break;

            case FIRING:
                if (stateTimer.getElapsedTimeSeconds() > TIME_TO_SHOOT) {
                    closeStopper();
                    next(nextState);
                }
                break;
        }
    }

    private void followOnce(PathChain path) {
        if (firstEnter) {
            follower.followPath(path, true);
            stateTimer.resetTimer();
            firstEnter = false;
        }
    }

    private boolean pathTimedOut() {
        return follower.isBusy() && stateTimer.getElapsedTimeSeconds() > MAX_PATH_TIMEOUT;
    }

    private boolean parkDone() {
        return !follower.isBusy() || stateTimer.getElapsedTimeSeconds() > MAX_PATH_TIMEOUT;
    }

    private boolean advanceWhenPathDone(PathState nextState) {
        if (!follower.isBusy()) {
            next(nextState);
            return true;
        }
        if (pathTimedOut()) {
            emergencyPark();
            return true;
        }
        return false;
    }

    private boolean shooterReady() {
        return Math.abs(shooter1.getVelocity() - SHOOTER_VELO_MAX) <= SHOOTER_READY_TOLERANCE
                && Math.abs(shooter2.getVelocity() - SHOOTER_VELO_MAX) <= SHOOTER_READY_TOLERANCE;
    }

    private double currentShootWarmupDelay() {
        return pathState == PathState.SHOOT_1 ? FIRST_SHOT_WARMUP_DELAY : FOLLOWUP_SHOT_WARMUP_DELAY;
    }

    private boolean canOpenStopper(double elapsedSeconds) {
        if (elapsedSeconds <= currentShootWarmupDelay()) return false;
        return shooterReady() || elapsedSeconds > SHOT_FORCE_OPEN_DELAY;
    }

    private void emergencyPark() {
        intake.setPower(0);
        transfer.setPower(0);
        emergencyExit = true;
        pathState = PathState.PARK;
        firstEnter = true;
    }

    private void next(PathState nextState) {
        pathState = nextState;
        firstEnter = true;
    }

    private void collectIntake() {
        intake.setPower(INTAKE_POWER_COLLECT);
        transfer.setPower(INTAKE_POWER_COLLECT);
        closeStopper();
    }

    private void keepIntakeRunning() {
        intake.setPower(INTAKE_POWER_HOLD);
        transfer.setPower(TRANSFER_POWER_HOLD);
        closeStopper();
    }

    private void openStopper() {
        stopper.setPosition(STOPPER_OPEN);
    }

    private void closeStopper() {
        stopper.setPosition(STOPPER_CLOSED);
    }

    private void holdTurelServos() {
        turel.setPosition(turelHoldPosition);
        turel1.setPosition(turel1HoldPosition);
    }

    private void setTurelShot1Position() {
        turelHoldPosition  = TUREL_SHOT_1_POSITION;
        turel1HoldPosition = TUREL_SHOT_1_POSITION;
    }

    private void setTurelShot2Position() {
        turelHoldPosition  = TUREL_SHOT_2_POSITION;
        turel1HoldPosition = TUREL_SHOT_2_POSITION;
    }

    private void setTurelParkPosition() {
        turelHoldPosition  = TUREL_PARK_POSITION;
        turel1HoldPosition = TUREL_PARK_POSITION;
    }

    private void updateTurelForCurrentState() {
        if (pathState == PathState.START_TO_SHOOT || pathState == PathState.SHOOT_1) {
            setTurelShot1Position();
        } else {
            setTurelShot2Position();
        }
    }

    private void stopAllMechanisms() {
        intake.setPower(0);
        transfer.setPower(0);
        shooter1.setVelocity(0);
        shooter2.setVelocity(0);
    }

    private void buildPaths() {
        startToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        goCollect1 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, collectControlPose1, collectPose1))
                .setLinearHeadingInterpolation(shootPose.getHeading(), collectPose1.getHeading())
                .build();

        return1 = follower.pathBuilder()
                .addPath(new BezierCurve(collectPose1, returnControlPose1, shootPose))
                .setLinearHeadingInterpolation(collectPose1.getHeading(), shootPose.getHeading())
                .build();

        goCollect2 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, collectPose2))
                .setLinearHeadingInterpolation(shootPose.getHeading(), collectPose2.getHeading())
                .build();

        return2 = follower.pathBuilder()
                .addPath(new BezierLine(collectPose2, shootPose))
                .setLinearHeadingInterpolation(collectPose2.getHeading(), shootPose.getHeading())
                .build();

        goCollect3 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, gatePose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), gatePose.getHeading())
                .build();

        goCollect35 = follower.pathBuilder()
                .addPath(new BezierCurve(gatePose, gateDeepControl, gateDeepPose))
                .setLinearHeadingInterpolation(gatePose.getHeading(), gateDeepPose.getHeading())
                .build();

        return3 = follower.pathBuilder()
                .addPath(new BezierLine(gateDeepPose, shootPose))
                .setLinearHeadingInterpolation(gateDeepPose.getHeading(), shootPose.getHeading())
                .build();

        goCollect4 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, gatePose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), gatePose.getHeading())
                .build();

        goCollect45 = follower.pathBuilder()
                .addPath(new BezierCurve(gatePose, gateDeepControl, gateDeepPose))
                .setLinearHeadingInterpolation(gatePose.getHeading(), gateDeepPose.getHeading())
                .build();

        return4 = follower.pathBuilder()
                .addPath(new BezierLine(gateDeepPose, shootPose))
                .setLinearHeadingInterpolation(gateDeepPose.getHeading(), shootPose.getHeading())
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, parkPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), parkPose.getHeading())
                .build();
    }
}
