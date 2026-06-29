package org.firstinspires.ftc.teamcode.configTamaraV3;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class turelBlue extends OpMode {
    private Servo xr;
    private Servo xl;

    private Limelight3A limelight;

    @Override
    public void init(){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8);
        limelight.start();

        xl = hardwareMap.get(Servo.class, "t1");
        xr = hardwareMap.get(Servo.class, "t2");

    }
    @Override
    public void loop(){
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()){
            double pos = 0.5 - llResult.getTx() * 0.0115;
            pos = Range.clip(pos, 0.0, 1.0);
            if (pos >= 0.95){
                pos = 0.1;
            }else if(pos <= 0.05){
                pos = 0.9;
            }
            xr.setPosition(pos);
            xl.setPosition(pos);

        }
    }
}
