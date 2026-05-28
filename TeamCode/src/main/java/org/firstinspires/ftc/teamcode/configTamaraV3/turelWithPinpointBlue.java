package org.firstinspires.ftc.teamcode.configTamaraV3;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class turelWithPinpointRed {

    private Servo t1, t2;

    GoBildaPinpointDriver pinpointDriver;

    private Limelight3A limelight;


    double targetPos= 0;

    double error = 0;

    boolean flag = true;
    double servoPos = 0.5;

    TelemetryPacket packet = new TelemetryPacket();
    FtcDashboard dashboard = FtcDashboard.getInstance();

    public void init(HardwareMap hwMap){
        limelight = hwMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        limelight.start();

        pinpointDriver = hwMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpointDriver.setOffsets(0,0, DistanceUnit.CM);

        pinpointDriver.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpointDriver.resetPosAndIMU();

        t1 = hwMap.get(Servo.class, "t1");
        t2 = hwMap.get(Servo.class, "t2");



    }


    public void turel(boolean g2dpup, boolean g2dl, double g2rt, double g2lt){
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()){
            double pos = 0.5 + llResult.getTx() * 0.01;
            pos = Range.clip(pos, 0.0, 1.0);
            /*if (pos >= 0.95){
                pos = 0.1;
            }else if(pos <= 0.05){
                pos = 0.9;
            }*/
            t1.setPosition(pos);
            t2.setPosition(pos);
            packet.put("tx: ", llResult.getTx());
            dashboard.sendTelemetryPacket(packet);


        }/*else{
            double robotHeadingRad = pinpointDriver.getHeading(AngleUnit.RADIANS);
            error = targetPos - robotHeadingRad;
            servoPos = 0.5 + error * 0.162;

            t1.setPosition(servoPos);
            t2.setPosition(servoPos);
        }*/




        /*pinpointDriver.update();
        if (g2dpup){
            flag = true;
            pinpointDriver.resetPosAndIMU();
        } else if (g2dl) {
            flag = false;
        }

        if (flag){
            double robotHeadingRad = pinpointDriver.getHeading(AngleUnit.RADIANS);
            error = targetPos - robotHeadingRad;
            servoPos = 0.5 + error * 0.162;

            t1.setPosition(servoPos);
            t2.setPosition(servoPos);
        }else {
            servoPos += (g2rt + g2lt) * 0.02;
            servoPos = Math.max(0, Math.min(1, servoPos));
            t1.setPosition(servoPos);
            t2.setPosition(servoPos);
        }*/




    }

}
