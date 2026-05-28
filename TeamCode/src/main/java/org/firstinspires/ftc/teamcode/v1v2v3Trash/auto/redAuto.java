package org.firstinspires.ftc.teamcode.v1v2v3Trash.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class redAuto extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    autoShoot shoot = new autoShoot();

    private boolean shotsTriger = false;
    private boolean intakeTriger = false;

    private Servo xl, xr;

    public enum PathState{
        Drive_StartPos_ShootPos,
        ShootPos_PreDopT,
        PreDopT_DopT,
        DopT_OpGate90,  
        Shoot_PreDopC,
        PreDopc_DopC,
        DopC_PreOpGate,
        PreOpGate_OpGate,
        OpGate_Shoot,
        Shoot_PreDopGate,
        PreDopGate_PreDopGate,
        OpGate90_ShootPos,
        ShootPos_PreDopU,
        PreDopU_DopU,
        DopU_ShootPos,
        PreDopGate_Gate,
        Gate_ShootPos

    }


    private final Pose startPose = new Pose(116.05351170568561, 132.13377926421404, Math.toRadians(37));
    private final Pose shootPose = new Pose(87, 85, Math.toRadians(15));

    private final Pose preDopT = new Pose(102, 85, Math.toRadians(0));
    private final Pose dopT = new Pose(130, 85, Math.toRadians(0));

    private final Pose preDopC = new Pose(100, 63, Math.toRadians(0));
    private final Pose dopC = new Pose(131, 63, Math.toRadians(0));

    private final Pose preOpGate = new Pose(119, 70, Math.toRadians(0));
    private final Pose OpGate = new Pose(130, 70, Math.toRadians(0));

    private final Pose park = new Pose(110, 80,Math.toRadians(90));

    private final Pose OpGate90 = new Pose(14, 70, Math.toRadians(90));

    private final Pose preDopU = new Pose(43, 35, Math.toRadians(180));
    private final Pose dopU = new Pose(10, 35, Math.toRadians(180));


    private PathChain driveStartPos2ShootPos, shootPos2PreDopT,
            takingDopT, tDop2OpGate90, opGate2shoot,
            ShootPos2preDopC, takingDopC,
            cDop2preGate, preGate2Gate,
            gate2Shoot, shootPos2PreDopU, takingDopU, uDop2Shoot,
            shoot2Park;
    public void buildPath(){
        driveStartPos2ShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        shootPos2PreDopT = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, preDopT))
                .setLinearHeadingInterpolation(shootPose.getHeading(), preDopT.getHeading())
                .build();
        takingDopT = follower.pathBuilder()
                .addPath(new BezierLine(preDopT, dopT))
                .setLinearHeadingInterpolation(preDopT.getHeading(), dopT.getHeading())
                .build();
        tDop2OpGate90 = follower.pathBuilder()
                .addPath(new BezierLine(dopT, OpGate90))
                .setLinearHeadingInterpolation(dopT.getHeading(), OpGate90.getHeading())
                .build();
        opGate2shoot = follower.pathBuilder()
                .addPath(new BezierLine(OpGate90, shootPose))
                .setLinearHeadingInterpolation(OpGate90.getHeading(), shootPose.getHeading())
                .build();
        ShootPos2preDopC = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, preDopC))
                .setLinearHeadingInterpolation(shootPose.getHeading(), preDopC.getHeading())
                .build();
        takingDopC = follower.pathBuilder()
                .addPath(new BezierLine(preDopC, dopC))
                .setLinearHeadingInterpolation(preDopC.getHeading(), dopC.getHeading())
                .build();
        cDop2preGate = follower.pathBuilder()
                .addPath(new BezierLine(dopC, preOpGate))
                .setLinearHeadingInterpolation(dopC.getHeading(), preOpGate.getHeading())
                .build();
        preGate2Gate = follower.pathBuilder()
                .addPath(new BezierLine(preOpGate, OpGate))
                .setLinearHeadingInterpolation(preOpGate.getHeading(), OpGate.getHeading())
                .build();
        gate2Shoot = follower.pathBuilder()
                .addPath(new BezierLine(OpGate, shootPose))
                .setLinearHeadingInterpolation(OpGate.getHeading(), shootPose.getHeading())
                .build();
        shoot2Park = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, park))
                .setLinearHeadingInterpolation(shootPose.getHeading(), park.getHeading())
                .build();
        shootPos2PreDopU = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, preDopU))
                .setLinearHeadingInterpolation(shootPose.getHeading(), preDopU.getHeading())
                .build();
        takingDopU = follower.pathBuilder()
                .addPath(new BezierLine(preDopU, dopU))
                .setLinearHeadingInterpolation(preDopU.getHeading(), dopU.getHeading())
                .build();
        uDop2Shoot = follower.pathBuilder()
                .addPath(new BezierLine(dopU, shootPose))
                .setLinearHeadingInterpolation(dopU.getHeading(), shootPose.getHeading())
                .build();
    }

    public void statePathUpdate(){
        switch (pathState){
            case Drive_StartPos_ShootPos:
                follower.followPath(driveStartPos2ShootPos, true);
                setPathState(PathState.Shoot_PreDopC);
                break;
            case ShootPos_PreDopT:
                if (!follower.isBusy()){
                    if (!shotsTriger){
                        shoot.FireShots(1);
                        shotsTriger = true;
                    } else if (shotsTriger && !shoot.isBusy()){
                        shotsTriger = false;
                        follower.followPath(shootPos2PreDopT, true);
                        setPathState(PathState.PreDopT_DopT);
                    }
                }
                break;
            case PreDopT_DopT:
                if (!follower.isBusy()) {
                    if (!intakeTriger){
                        shoot.InShots(1);
                        intakeTriger = true;
                    } else if (intakeTriger) {
                        follower.setMaxPower(0.9);
                        intakeTriger = false;
                        follower.followPath(takingDopT, true);
                        setPathState(PathState.DopT_OpGate90);
                    }
                }
                break;
            case DopT_OpGate90:
                if (!follower.isBusy()){
                    follower.setMaxPower(1);
                    follower.followPath(tDop2OpGate90, true);
                    setPathState(PathState.OpGate90_ShootPos);
                }
                break;
            case OpGate90_ShootPos:
                if (!follower.isBusy()){
                    follower.followPath(opGate2shoot, true);
                    setPathState(PathState.ShootPos_PreDopU);
                }
            case Shoot_PreDopC:
                if (!follower.isBusy()){
                    if (!shotsTriger){
                        shoot.FireShots(1);
                        shotsTriger = true;
                    } else if (shotsTriger && !shoot.isBusy()) {
                        shotsTriger = false;
                        follower.followPath(ShootPos2preDopC, true);
                        setPathState(PathState.PreDopc_DopC);
                    }
                }
                break;
            case PreDopc_DopC:
                if (!follower.isBusy()){
                    if (!intakeTriger){
                        shoot.InShots(1);
                        intakeTriger = true;
                    } else if (intakeTriger) {
                        follower.setMaxPower(0.9);
                        intakeTriger = false;
                        follower.followPath(takingDopC, true);
                        setPathState(PathState.DopC_PreOpGate);
                    }
                }
                break;
            case DopC_PreOpGate:
                if (!follower.isBusy()){
                    follower.followPath(cDop2preGate, true);
                    setPathState(PathState.PreOpGate_OpGate);
                }
                break;
            case PreOpGate_OpGate:
                if (!follower.isBusy()){
                    follower.setMaxPower(1.0);
                    follower.followPath(preGate2Gate, true);
                    setPathState(PathState.OpGate_Shoot);
                }
                break;
            case OpGate_Shoot:
                if (!follower.isBusy()){
                    follower.followPath(gate2Shoot, true);
                    setPathState(PathState.ShootPos_PreDopT);
                }
                break;
            case Shoot_PreDopGate:
                if (!follower.isBusy()){
                    if (!shotsTriger){
                        shoot.FireShots(1);
                        shotsTriger = true;
                    } else if (shotsTriger && !shoot.isBusy()) {
                        shotsTriger = false;
                        follower.followPath(shoot2Park, true);
                        setPathState(PathState.PreDopGate_PreDopGate);
                    }
                }
                break;
            case ShootPos_PreDopU:
                if (!follower.isBusy()){
                    if (!shotsTriger){
                        shoot.FireShots(1);
                        shotsTriger = true;
                    } else if (shotsTriger && !shoot.isBusy()) {
                        shotsTriger = false;
                        follower.followPath(shootPos2PreDopU, true);
                        setPathState(PathState.PreDopU_DopU);
                    }
                }
                break;
            case PreDopU_DopU:
                if (!follower.isBusy()){
                    if (!intakeTriger){
                        shoot.InShots(1);
                        intakeTriger = true;
                    } else if (intakeTriger) {
                        intakeTriger = false;
                        follower.followPath(takingDopU, true);
                        setPathState(PathState.DopU_ShootPos);
                    }
                }
                break;
            case DopU_ShootPos:
                if (!follower.isBusy()){
                    if (!shotsTriger){
                        shoot.FireShots(1);
                        shotsTriger = true;
                    } else if (shotsTriger && !shoot.isBusy()) {
                        shotsTriger = false;
                        follower.followPath(shoot2Park, true);
                    }
                }
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


        buildPath();
        follower.setPose(startPose);
    }
    @Override
    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
        xl.setPosition(0.18);
        xr.setPosition(0.18);
    }

    @Override
    public void loop() {
        follower.update();
        shoot.update();
        shoot.intake();
        statePathUpdate();
    }
}
