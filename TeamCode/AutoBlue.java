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
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@Autonomous(name = "ZHANDOS TOP", group = "Digital")
public class AutoRed extends OpMode {

    public static double SHOOTER_POWER = 0.8;
    public static double INTAKE_POWER_COLLECT = 1.0;
    public static double INTAKE_POWER_HOLD = 1.0;
    public static double INTAKE_POWER_SHOOT = 1.0;
    public static double TRANSFER_POWER_SHOOT = 1.0;

    public static double STOPPER_OPEN = 0.05;
    public static double STOPPER_CLOSED = 0.45;

    public static double SHOOT_WARMUP_DELAY = 1.0;
    public static double TIME_TO_SHOOT = 0.8;

    public static double FIRST_SHOOT_DELAY = 0.5;
    public static double TIME_FOR_GATE_INTAKE = 0.6;
    public static double EMERGENCY_EXIT_TIME = 29.0;

    private Follower follower;
    private DcMotorEx intake;
    private DcMotorEx shooter1;
    private DcMotorEx transfer;
    private DcMotorEx turel;
    private Servo stopper;

    private double currentShooterPower = 1.0;

    private enum ShootSubState {
        WARMUP,
        FIRING
    }
    private ShootSubState shootSubState = ShootSubState.WARMUP;

    private enum PathState {
        START_TO_SHOOT,
        SHOOT_1,
        GO_COLLECT_1,
        COLLECT_1,
        RETURN_1,
        SHOOT_2,
        COLLECT_2,
        RETURN_2,
        SHOOT_3,
        GO_COLLECT_3,
        RETURN_3,
        SHOOT_4,
        GO_COLLECT_4,
        COLLECT_4,
        RETURN_4,
        SHOOT_5,
        PARK,
        END
    }

    private PathState pathState = PathState.START_TO_SHOOT;
    private final Timer stateTimer = new Timer();
    private final Timer gateWaitTimer = new Timer();
    private boolean firstEnter = true;
    private boolean emergencyExit = false;
    private boolean gatePathDone = false;

    private PathChain startToShoot;
    private PathChain goCollect1;
    private PathChain collect1;
    private PathChain return1;
    private PathChain collect2;
    private PathChain return2;
    private PathChain goCollect3;
    private PathChain return3;
    private PathChain goCollect4;
    private PathChain collect4;
    private PathChain return4;
    private PathChain park;

    // Обновлено под твою новую начальную точку из Path Generator
    private final Pose startPose = new Pose(108, 133, Math.toRadians(37));
    private final Pose shootPose = new Pose(91, 93, Math.toRadians(50));
    private final Pose parkPose = new Pose(144-36, 58, Math.toRadians(0));

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        turel = hardwareMap.get(DcMotorEx.class, "turel");
        stopper = hardwareMap.get(Servo.class, "stopper");

        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        transfer.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
        turel.setDirection(DcMotorSimple.Direction.FORWARD);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        turel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        buildPaths();
        stopAllMechanisms();
        closeStopper();
    }

    @Override
    public void start() {
        pathState = PathState.START_TO_SHOOT;
        firstEnter = true;
        emergencyExit = false;
        currentShooterPower = 1.0;
        stateTimer.resetTimer();
        closeStopper();
        shooter1.setPower(currentShooterPower);
        turel.setPower(currentShooterPower);
    }

    @Override
    public void loop() {
        follower.update();
        updateFSM();

        telemetry.addData("State", pathState);
        telemetry.addData("Shoot SubState", shootSubState);
        telemetry.addData("Current Shooter Power", currentShooterPower);
        telemetry.addData("Runtime", getRuntime());
        telemetry.addData("Pose", follower.getPose());
        telemetry.addData("Intake", intake.getPower());
        telemetry.addData("Transfer", transfer.getPower());
        telemetry.addData("Shooter1", shooter1.getPower());
        telemetry.addData("Turel", turel.getPower());
        telemetry.addData("Stopper", stopper.getPosition());
        telemetry.update();
    }

    private void updateFSM() {
        if (!emergencyExit && getRuntime() > EMERGENCY_EXIT_TIME) {
            emergencyExit = true;
            pathState = PathState.PARK;
            firstEnter = true;
        }

        switch (pathState) {
            case START_TO_SHOOT:
                followOnce(startToShoot);
                holdIntake();
                shooter1.setPower(currentShooterPower);
                turel.setPower(currentShooterPower);
                if (pathDoneAfter(FIRST_SHOOT_DELAY)) {
                    next(PathState.SHOOT_1);
                }
                break;

            case SHOOT_1:
                shootFor(PathState.GO_COLLECT_1);
                break;

            case GO_COLLECT_1:
                currentShooterPower = 0.60;
                followOnce(goCollect1);
                collectIntake();
                if (pathDone()) next(PathState.COLLECT_1);
                break;

            case COLLECT_1:
                followOnce(collect1);
                collectIntake();
                if (pathDone()) next(PathState.RETURN_1);
                break;

            case RETURN_1:
                followOnce(return1);
                holdIntake();
                if (pathDone()) next(PathState.SHOOT_2);
                break;

            case SHOOT_2:
                shootFor(PathState.COLLECT_2);
                break;

            case COLLECT_2:
                followOnce(collect2);
                collectIntake();
                if (pathDone()) next(PathState.RETURN_2);
                break;

            case RETURN_2:
                followOnce(return2);
                holdIntake();
                if (pathDone()) next(PathState.SHOOT_3);
                break;

            case SHOOT_3:
                shootFor(PathState.GO_COLLECT_3);
                break;

            case GO_COLLECT_3:
                if (firstEnter) {
                    follower.followPath(goCollect3, true);
                    gatePathDone = false;
                    stateTimer.resetTimer();
                    firstEnter = false;
                }
                collectIntake();
                if (!follower.isBusy() && !gatePathDone) {
                    gateWaitTimer.resetTimer();
                    gatePathDone = true;
                }
                if (gatePathDone && gateWaitTimer.getElapsedTimeSeconds() > TIME_FOR_GATE_INTAKE) {
                    next(PathState.RETURN_3);
                }
                break;

            case RETURN_3:
                followOnce(return3);
                holdIntake();
                if (pathDone()) next(PathState.SHOOT_4);
                break;

            case SHOOT_4:
                shootFor(PathState.GO_COLLECT_4);
                break;

            case GO_COLLECT_4:
                followOnce(goCollect4);
                collectIntake();
                if (pathDone()) next(PathState.COLLECT_4);
                break;

            case COLLECT_4:
                followOnce(collect4);
                collectIntake();
                if (pathDone()) next(PathState.RETURN_4);
                break;

            case RETURN_4:
                followOnce(return4);
                holdIntake();
                if (pathDone()) next(PathState.SHOOT_5);
                break;

            case SHOOT_5:
                shootFor(PathState.PARK);
                break;

            case PARK:
                followOnce(park);
                holdIntake();
                if (pathDone()) next(PathState.END);
                break;

            case END:
                stopAllMechanisms();
                closeStopper();
                break;
        }
    }

    private void shootFor(PathState nextState) {
        if (firstEnter) {
            closeStopper();
            shooter1.setPower(currentShooterPower);
            turel.setPower(currentShooterPower);

            shootSubState = ShootSubState.WARMUP;
            stateTimer.resetTimer();
            firstEnter = false;
        }

        switch (shootSubState) {
            case WARMUP:
                closeStopper();
                intake.setPower(0);
                transfer.setPower(0);

                if (stateTimer.getElapsedTimeSeconds() > SHOOT_WARMUP_DELAY) {
                    openStopper();
                    intake.setPower(Range.clip(INTAKE_POWER_COLLECT, -1.0, 1.0));
                    transfer.setPower(Range.clip(TRANSFER_POWER_SHOOT, -1.0, 1.0));

                    shootSubState = ShootSubState.FIRING;
                    stateTimer.resetTimer();
                }
                break;

            case FIRING:
                if (stateTimer.getElapsedTimeSeconds() > TIME_TO_SHOOT) {
                    closeStopper();
                    transfer.setPower(0);
                    intake.setPower(0);
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

    private boolean pathDone() {
        return !follower.isBusy();
    }

    private boolean pathDoneAfter(double seconds) {
        return !follower.isBusy() && stateTimer.getElapsedTimeSeconds() > seconds;
    }

    private void next(PathState nextState) {
        pathState = nextState;
        firstEnter = true;
    }

    private void collectIntake() {
        intake.setPower(Range.clip(INTAKE_POWER_COLLECT, -1.0, 1.0));
        transfer.setPower(Range.clip(INTAKE_POWER_COLLECT, -1.0, 1.0));
        closeStopper();
    }

    private void holdIntake() {
        intake.setPower(Range.clip(INTAKE_POWER_HOLD, -1.0, 1.0));
        transfer.setPower(-0.15);
        closeStopper();
    }

    private void openStopper() {
        stopper.setPosition(STOPPER_OPEN);
    }

    private void closeStopper() {
        stopper.setPosition(STOPPER_CLOSED);
    }

    private void stopAllMechanisms() {
        intake.setPower(0);
        transfer.setPower(0);
        shooter1.setPower(0);
        turel.setPower(0);
    }

    private void buildPaths() {
        // Замена путей строго по твоим скриншотам из генератора траекторий

        // Starting Point: X: 108, Y: 133
        final Pose generatorStartPose = new Pose(108, 133, Math.toRadians(37));

        // Path 1
        startToShoot = follower.pathBuilder()
                .addPath(new BezierLine(generatorStartPose, new Pose(94.35, 94.41)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // Path 2
        goCollect1 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(94.35, 94.41), new Pose(94, 94)))
                .setConstantHeadingInterpolation(Math.toRadians(45))
                .build();

        // Path 3
        collect1 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(94, 94), new Pose(93.90, 82.24)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // Path 4 (Ошибка .setMaxPower(0.4) полностью удалена)
        return1 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(93.90, 82.24), new Pose(123.8, 81.82)))
                .setTangentHeadingInterpolation()
                .build();

        // Path 5
        collect2 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(123.8, 81.82), new Pose(94.66, 94.38)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                .build();

        // Path 6
        return2 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(94.66, 94.38), new Pose(94.3, 56.90)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // Path 7 из генератора
        goCollect3 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(94.3, 56.90), new Pose(123, 58.41)))
                .setTangentHeadingInterpolation()
                .build();


        // --- ОСТАЛЬНЫЕ СТАРЫЕ МАРШРУТЫ (ИХ НЕ ТРОГАЛ) ---

        return3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(123, 58.41), new Pose(103, 70), shootPose))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(50))
                .build();

        goCollect4 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, new Pose(103, 70)))
                .setLinearHeadingInterpolation(Math.toRadians(50), Math.toRadians(0))
                .build();

        collect4 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(103, 70), new Pose(133, 60)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        return4 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(133, 60), shootPose))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(50))
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, parkPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), parkPose.getHeading())
                .build();
    }
}