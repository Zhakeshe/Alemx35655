package org.firstinspires.ftc.teamcode.configTamaraV3;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Config
public class turelWithPinpointBlue {

    private Servo t1, t2;




    double targetPos= 0.5;

    double error = 0;

    boolean flag = true;
    boolean flag2 = false;
    double pos = 0;

    public static double sens = -0.002;


    TelemetryPacket packet = new TelemetryPacket();
    FtcDashboard dashboard = FtcDashboard.getInstance();

    public void init(HardwareMap hwMap){
        //limelight = hwMap.get(Limelight3A.class, "limelight");
        //limelight.pipelineSwitch(1);
        //limelight.start();

        t1 = hwMap.get(Servo.class, "t1");
        t2 = hwMap.get(Servo.class, "t2");



    }


    public void turel(boolean g2dpup, boolean g2dr, boolean g2dl, boolean g2dd, double g2rt, double g2lt){

        pos = pos + (g2lt + g2rt) * 0.015;
        t1.setPosition(pos);
        t2.setPosition(pos);
        /*if (flag){
            double robotHeadingRad = pinpointDriver.getHeading(AngleUnit.RADIANS);
            error = targetPos - robotHeadingRad;
            pos = 0.5 + error * 0.18;
            t1.setPosition(pos);
            t2.setPosition(pos);
        }else if(!flag && flag2){
            LLResult llResult = limelight.getLatestResult();
            if (llResult != null && llResult.isValid()){
                pos = (0.5 - llResult.getTx() * sens) - (-llResult.getTx() * 0.00055);
                pos = Range.clip(pos, 0.0, 1.0);
                t1.setPosition(pos);
                t2.setPosition(pos);
                packet.put("tx: ", llResult.getTx());
                packet.put("tPos: ", t1.getPosition());
                dashboard.sendTelemetryPacket(packet);
            }
        } else if (!flag && !flag) {
            pos = pos + (g2lt + g2rt) * 0.015;
            t1.setPosition(pos);
            t2.setPosition(pos);
        }*/

    }/*else{
            double robotHeadingRad = pinpointDriver.getHeading(AngleUnit.RADIANS);
            error = targetPos - robotHeadingRad;
            pos = 0.5 + error * 0.162;

            t1.setPosition(pos);
            t2.setPosition(pos);
        }*/




}



