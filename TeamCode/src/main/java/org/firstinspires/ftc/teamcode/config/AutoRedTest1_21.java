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

    @Configurable
    @Autonomous(name = "AutoRedTest1-21", group = "Digital")
    public class AutoRedTest1_21 extends OpMode {

        public static double SHOOTER_VELO_MAX = 1200.0;
        public static double SHOOTER_VELO_IDLE = 600.0;

        public static double INTAKE_POWER_COLLECT = 1.0;
        public static double INTAKE_POWER_HOLD    = 1.0;
        public static double TRANSFER_POWER_HOLD  = 1.0;
        public static double INTAKE_POWER_SHOOT   = 1.0;
        public static double TRANSFER_POWER_SHOOT = 1.0;

        public static double STOPPER_OPEN   = 0.08;
        public static double STOPPER_CLOSED = 0.48;

        public static double SHOOTER_READY_TOLERANCE    = 30.0;
        public static double SHOOTER_STABLE_WINDOW      = 0.10;

        public static double FIRST_SHOT_WARMUP_DELAY    = 0.20;
        public static double FOLLOWUP_SHOT_WARMUP_DELAY = 0.12;
        public static double SHOT_FORCE_OPEN_DELAY      = 0.60;

        public static double TIME_TO_SHOOT = 0.5;

        public static double TUREL_SHOT_1_POSITION = 0.5;
        public static double TUREL_SHOT_2_POSITION = 0.5;
        public static double TUREL_PARK_POSITION   = 0.5;

        public static double TIME_TO_COLLECT_WAIT = 0.35;

        public static double FIRST_SHOOT_DELAY   = 0.95;
        public static double EMERGENCY_EXIT_TIME = 29.0;
        public static double MAX_PATH_TIMEOUT    = 6.0;

        private Follower  follower;
        private DcMotorEx intake;
        private DcMotorEx shooter1;
        private DcMotorEx shooter2;
        private DcMotorEx transfer;
        private Servo     turel;
        private Servo     turel1;
        private Servo     stopper;
        private double    turelHoldPosition;
        private double    turel1HoldPosition;

        private final Timer shooterStableTimer = new Timer();
        private boolean     shooterWasReady    = false;

        private double waitCollectStartTime = -1.0;

        private enum ShootSubState { WARMUP, FIRING }
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
            GO_COLLECT_5,
            WAIT_COLLECT_5,
            GO_COLLECT_55,
            RETURN_5,
            SHOOT_6,
            GO_COLLECT_6,
            WAIT_COLLECT_6,
            GO_COLLECT_65,
            RETURN_6,
            SHOOT_7,
            PARK,
            END
        }

        private PathState   pathState     = PathState.START_TO_SHOOT;
        private final Timer stateTimer    = new Timer();
        private boolean     firstEnter    = true;
        private boolean     emergencyExit = false;

        private PathChain startToShoot;
        private PathChain goCollect1, return1;
        private PathChain goCollect2, return2;
        private PathChain goCollect3, return3;
        private PathChain goCollect4, return4;
        private PathChain goCollect5, return5;
        private PathChain goCollect6, return6;
        private PathChain goCollect35;
        private PathChain goCollect45;
        private PathChain goCollect55;
        private PathChain goCollect65;
        private PathChain park;

        private final Pose startPose = new Pose(105, 134, Math.toRadians(0));
        private final Pose shootPose = new Pose(93.2111801242236, 83.95496894409939, Math.toRadians(0));

        private final Pose collectPose1        = new Pose(124, 64, Math.toRadians(0));
        private final Pose collectControlPose1 = new Pose(84.59549689440996, 62, Math.toRadians(0));
        private final Pose returnControlPose1  = new Pose(105.24245016260936, 71.7388417843294, Math.toRadians(0));

        private final Pose collectPose2 = new Pose(119, 93, Math.toRadians(0));

        private final Pose gatePose        = new Pose(127, 66.5, Math.toRadians(20));
        private final Pose gateDeepPose    = new Pose(130.22558355503236, 58.46836679847239, Math.toRadians(0));
        private final Pose gateDeepControl = new Pose(124.81271413776464, 58.211667871285876, Math.toRadians(0));

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
            pathState            = PathState.START_TO_SHOOT;
            firstEnter           = true;
            emergencyExit        = false;
            shooterWasReady      = false;
            waitCollectStartTime = -1.0;

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

            if (pathState == PathState.END) {
                stopAllMechanisms();
                holdTurelServos();
                closeStopper();
                telemetry.addData("State", "END / IDLE");
                telemetry.update();
                return;
            }

            updateFSM();

            telemetry.addData("State", pathState);
            telemetry.addData("SubState", shootSubState);
            telemetry.addData("Shooter1", "%.1f", shooter1.getVelocity());
            telemetry.addData("Shooter2", "%.1f", shooter2.getVelocity());
            telemetry.addData("Diff1", "%.1f", SHOOTER_VELO_MAX - shooter1.getVelocity());
            telemetry.addData("Diff2", "%.1f", SHOOTER_VELO_MAX - shooter2.getVelocity());
            telemetry.addData("ShooterReady", shooterReady());
            telemetry.addData("StableFor", "%.3f s", shooterWasReady ? shooterStableTimer.getElapsedTimeSeconds() : 0.0);
            telemetry.addData("WaitCollect", "%.2f s", waitCollectStartTime >= 0 ? getRuntime() - waitCollectStartTime : -1.0);
            telemetry.addData("StateTimer", "%.2f s", stateTimer.getElapsedTimeSeconds());
            telemetry.addData("EmergencyExit", emergencyExit);
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
                    if (firstEnter) {
                        waitCollectStartTime = getRuntime();
                        firstEnter = false;
                    }
                    collectIntake();
                    if (getRuntime() - waitCollectStartTime > TIME_TO_COLLECT_WAIT) {
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
                    if (firstEnter) {
                        waitCollectStartTime = getRuntime();
                        firstEnter = false;
                    }
                    collectIntake();
                    if (getRuntime() - waitCollectStartTime > TIME_TO_COLLECT_WAIT) {
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
                    shootFor(PathState.GO_COLLECT_5);
                    break;

                case GO_COLLECT_5:
                    followOnce(goCollect5);
                    collectIntake();
                    advanceWhenPathDone(PathState.WAIT_COLLECT_5);
                    break;

                case WAIT_COLLECT_5:
                    if (firstEnter) {
                        waitCollectStartTime = getRuntime();
                        firstEnter = false;
                    }
                    collectIntake();
                    if (getRuntime() - waitCollectStartTime > TIME_TO_COLLECT_WAIT) {
                        next(PathState.GO_COLLECT_55);
                    }
                    break;

                case GO_COLLECT_55:
                    followOnce(goCollect55);
                    collectIntake();
                    advanceWhenPathDone(PathState.RETURN_5);
                    break;

                case RETURN_5:
                    followOnce(return5);
                    keepIntakeRunning();
                    advanceWhenPathDone(PathState.SHOOT_6);
                    break;

                case SHOOT_6:
                    shootFor(PathState.GO_COLLECT_6);
                    break;

                case GO_COLLECT_6:
                    followOnce(goCollect6);
                    collectIntake();
                    advanceWhenPathDone(PathState.WAIT_COLLECT_6);
                    break;

                case WAIT_COLLECT_6:
                    if (firstEnter) {
                        waitCollectStartTime = getRuntime();
                        firstEnter = false;
                    }
                    collectIntake();
                    if (getRuntime() - waitCollectStartTime > TIME_TO_COLLECT_WAIT) {
                        next(PathState.GO_COLLECT_65);
                    }
                    break;

                case GO_COLLECT_65:
                    followOnce(goCollect65);
                    collectIntake();
                    advanceWhenPathDone(PathState.RETURN_6);
                    break;

                case RETURN_6:
                    followOnce(return6);
                    keepIntakeRunning();
                    advanceWhenPathDone(PathState.SHOOT_7);
                    break;

                case SHOOT_7:
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
                shootSubState   = ShootSubState.WARMUP;
                shooterWasReady = false;
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
                    shooter1.setVelocity(SHOOTER_VELO_MAX);
                    shooter2.setVelocity(SHOOTER_VELO_MAX);

                    if (stateTimer.getElapsedTimeSeconds() > TIME_TO_SHOOT) {
                        closeStopper();
                        next(nextState);
                    }
                    break;
            }
        }

        private boolean canOpenStopper(double elapsedSeconds) {
            if (elapsedSeconds <= currentShootWarmupDelay()) {
                shooterWasReady = false;
                return false;
            }

            if (elapsedSeconds > SHOT_FORCE_OPEN_DELAY) {
                return true;
            }

            if (shooterReady()) {
                if (!shooterWasReady) {
                    shooterStableTimer.resetTimer();
                    shooterWasReady = true;
                }
                return shooterStableTimer.getElapsedTimeSeconds() >= SHOOTER_STABLE_WINDOW;
            } else {
                shooterWasReady = false;
                return false;
            }
        }

        private boolean shooterReady() {
            double v1 = shooter1.getVelocity();
            double v2 = shooter2.getVelocity();
            return v1 > 0 && v2 > 0
                    && Math.abs(v1 - SHOOTER_VELO_MAX) <= SHOOTER_READY_TOLERANCE
                    && Math.abs(v2 - SHOOTER_VELO_MAX) <= SHOOTER_READY_TOLERANCE;
        }

        private double currentShootWarmupDelay() {
            return pathState == PathState.SHOOT_1 ? FIRST_SHOT_WARMUP_DELAY : FOLLOWUP_SHOT_WARMUP_DELAY;
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

        private void emergencyPark() {
            intake.setPower(0);
            transfer.setPower(0);
            emergencyExit = true;
            pathState  = PathState.PARK;
            firstEnter = true;
        }

        private void next(PathState nextState) {
            pathState  = nextState;
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

        private void openStopper()  { stopper.setPosition(STOPPER_OPEN);   }
        private void closeStopper() { stopper.setPosition(STOPPER_CLOSED); }

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

            goCollect5 = follower.pathBuilder()
                    .addPath(new BezierLine(shootPose, gatePose))
                    .setLinearHeadingInterpolation(shootPose.getHeading(), gatePose.getHeading())
                    .build();

            goCollect55 = follower.pathBuilder()
                    .addPath(new BezierCurve(gatePose, gateDeepControl, gateDeepPose))
                    .setLinearHeadingInterpolation(gatePose.getHeading(), gateDeepPose.getHeading())
                    .build();

            return5 = follower.pathBuilder()
                    .addPath(new BezierLine(gateDeepPose, shootPose))
                    .setLinearHeadingInterpolation(gateDeepPose.getHeading(), shootPose.getHeading())
                    .build();

            goCollect6 = follower.pathBuilder()
                    .addPath(new BezierLine(shootPose, gatePose))
                    .setLinearHeadingInterpolation(shootPose.getHeading(), gatePose.getHeading())
                    .build();

            goCollect65 = follower.pathBuilder()
                    .addPath(new BezierCurve(gatePose, gateDeepControl, gateDeepPose))
                    .setLinearHeadingInterpolation(gatePose.getHeading(), gateDeepPose.getHeading())
                    .build();

            return6 = follower.pathBuilder()
                    .addPath(new BezierLine(gateDeepPose, shootPose))
                    .setLinearHeadingInterpolation(gateDeepPose.getHeading(), shootPose.getHeading())
                    .build();

            park = follower.pathBuilder()
                    .addPath(new BezierLine(shootPose, parkPose))
                    .setLinearHeadingInterpolation(shootPose.getHeading(), parkPose.getHeading())
                    .build();
        }
    }
