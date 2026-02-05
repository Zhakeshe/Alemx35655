package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.batysQual.RedAutoMain;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class blueAuto extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    autoShoot shoot = new autoShoot();

    private boolean shotsTriger = false;
    private boolean intakeTriger = false;

    private Servo xl, xr;

    public enum PathState{
        Drive_StartPos_ShootPos,
        ShootPos_TakingDop1,
        ShootPos_Going_Dop1,
        Taking_Dop1,
        Dop1_OpenGate,
        OpenGate_ShootPos,
        ShootPos_Going_Dop2,
        Dop2_TakingDop2,
        TakingDop2_ShootPos,
        ShootPos_GoingGate,
        GoingGate_OpeningGate
    }


    private final Pose startPose = new Pose(27.94648829431439, 132.13377926421404, Math.toRadians(143));
    private final Pose ShootPose = new Pose(57, 85, Math.toRadians(165));
    private final Pose Going2Dop1Pos = new Pose(44.49760765550239, 63, Math.toRadians(180));
    private final Pose TakingDop1Pos = new Pose(7, 63, Math.toRadians(180));
    private final Pose GateOpen = new Pose(50, 75, Math.toRadians(160));
    private final Pose Going2Dop2 = new Pose(44.49760765550239, 63, Math.toRadians(180));
    private final Pose TakingDop2 = new Pose(10, 63, Math.toRadians(140));
    private final Pose Going2Gate = new Pose(50, 75, Math.toRadians(130));
    private final Pose OpeningGate = new Pose(10, 63, Math.toRadians(140));

    private PathChain driveStartPos2ShootPos, shootPos2GoingDop1,
            going2takingdop1, takingDop2Gate,
            gatePos2ShootPosAfterDop1, shootPos2GoingDop2,
            going2TakingDop2, dop2Pos2ShootPos,
            shootPos2OpeningGate, going2open;
    public void buildPath(){
        driveStartPos2ShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, ShootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), ShootPose.getHeading())
                .build();
        shootPos2GoingDop1 = follower.pathBuilder()
                .addPath(new BezierLine(ShootPose, Going2Dop1Pos))
                .setLinearHeadingInterpolation(ShootPose.getHeading(), Going2Dop1Pos.getHeading())
                .build();
        going2takingdop1 = follower.pathBuilder()
                .addPath(new BezierLine(Going2Dop1Pos, TakingDop1Pos))
                .setLinearHeadingInterpolation(Going2Dop1Pos.getHeading(), TakingDop1Pos.getHeading())
                .build();
        takingDop2Gate = follower.pathBuilder()
                .addPath(new BezierLine(TakingDop1Pos, GateOpen))
                .setLinearHeadingInterpolation(TakingDop1Pos.getHeading(), GateOpen.getHeading())
                .build();
        gatePos2ShootPosAfterDop1 = follower.pathBuilder()
                .addPath(new BezierLine(GateOpen, ShootPose))
                .setLinearHeadingInterpolation(GateOpen.getHeading(), ShootPose.getHeading())
                .build();
        shootPos2GoingDop2 = follower.pathBuilder()
                .addPath(new BezierLine(ShootPose, Going2Dop2))
                .setLinearHeadingInterpolation(ShootPose.getHeading(), Going2Dop2.getHeading())
                .build();
        going2TakingDop2 = follower.pathBuilder()
                .addPath(new BezierLine(Going2Dop2, TakingDop2))
                .setLinearHeadingInterpolation(Going2Dop2.getHeading(), TakingDop2.getHeading())
                .build();
        dop2Pos2ShootPos = follower.pathBuilder()
                .addPath(new BezierLine(TakingDop2, ShootPose))
                .setLinearHeadingInterpolation(TakingDop2.getHeading(), ShootPose.getHeading())
                .build();
        shootPos2OpeningGate = follower.pathBuilder()
                .addPath(new BezierLine(ShootPose, Going2Gate))
                .setLinearHeadingInterpolation(ShootPose.getHeading(), Going2Gate.getHeading())
                .build();
        going2open = follower.pathBuilder()
                .addPath(new BezierLine(Going2Gate, OpeningGate))
                .setLinearHeadingInterpolation(Going2Gate.getHeading(), OpeningGate.getHeading())
                .build();
    }

    public void statePathUpdate(){
        switch (pathState){
            case Drive_StartPos_ShootPos:
                follower.followPath(driveStartPos2ShootPos, true);
                setPathState(PathState.ShootPos_TakingDop1);
                break;
            case ShootPos_TakingDop1:
                if (!follower.isBusy()){
                    if (!shotsTriger){
                        shoot.FireShots(1);
                        shotsTriger = true;
                    } else if (shotsTriger && !shoot.isBusy()) {
                        shotsTriger = false;
                        follower.followPath(shootPos2GoingDop1, true);
                        setPathState(PathState.ShootPos_Going_Dop1);
                    }
                }
                break;
            case ShootPos_Going_Dop1:
                if (!follower.isBusy()) {
                    if (!intakeTriger){
                        shoot.InShots(1);
                        intakeTriger = true;
                    } else if (intakeTriger && !shoot.isBusy()) {
                        follower.setMaxPower(0.7);
                        intakeTriger = false;
                        follower.followPath(going2takingdop1, true);
                        setPathState(PathState.Taking_Dop1);
                    }
                }
                break;
            case Taking_Dop1:
                if (!follower.isBusy()){
                    follower.setMaxPower(1);
                    follower.followPath(takingDop2Gate, true);
                    setPathState(PathState.Dop1_OpenGate);
                }
                break;
            case Dop1_OpenGate:
                if (!follower.isBusy()){
                    follower.followPath(gatePos2ShootPosAfterDop1, true);
                    setPathState(PathState.OpenGate_ShootPos);
                }
                break;
            case OpenGate_ShootPos:
                if (!follower.isBusy()){
                    if (!shotsTriger){
                        shoot.FireShots(1);
                        shotsTriger = true;
                    } else if (shotsTriger && !shoot.isBusy()) {
                        shotsTriger = false;
                        follower.followPath(shootPos2GoingDop2, true);
                        setPathState(PathState.ShootPos_Going_Dop2);
                    }
                }
                break;
            case ShootPos_Going_Dop2:
                if (!follower.isBusy()){
                    if (!intakeTriger){
                        shoot.InShots(1);
                        intakeTriger = true;
                    } else if (intakeTriger && !shoot.isBusy()) {
                        intakeTriger = false;
                        follower.followPath(going2TakingDop2, true);
                        setPathState(PathState.Dop2_TakingDop2);
                    }
                }
                break;
            case Dop2_TakingDop2:
                if (!follower.isBusy()){
                    follower.followPath(dop2Pos2ShootPos, true);
                    setPathState(PathState.TakingDop2_ShootPos);
                }
                break;
            case TakingDop2_ShootPos:
                if (!follower.isBusy()){
                    if (!shotsTriger){
                        shoot.FireShots(1);
                        shotsTriger = true;
                    } else if (shotsTriger && !shoot.isBusy()) {
                        shotsTriger = false;
                        follower.followPath(shootPos2OpeningGate, true);
                        setPathState(PathState.ShootPos_GoingGate);
                    }
                }
                break;
            case ShootPos_GoingGate:
                if (!follower.isBusy()){
                    if (!intakeTriger){
                        shoot.InShots(1);
                        intakeTriger = true;
                    } else if (intakeTriger && !shoot.isBusy()) {
                        intakeTriger = false;
                        follower.followPath(going2open, true);
                    }
                }
                break;
            default:
                break;
        }
    }

    public void setPathState(PathState newState){
        pathState = newState;
        pathTimer.resetTimer();
    }

    PathState pathState;


    @Override
    public void init() {
        pathState = PathState.Drive_StartPos_ShootPos;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        //
        shoot.init(hardwareMap);

        xl = hardwareMap.get(Servo.class, "xl");
        xr = hardwareMap.get(Servo.class, "xr");
        xl.setPosition(0.225);
        xr.setPosition(0.225);


        buildPath();
        follower.setPose(startPose);
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {
        follower.update();
        shoot.update();
        shoot.intake();
        statePathUpdate();
    }
}
