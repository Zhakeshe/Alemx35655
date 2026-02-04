package org.firstinspires.ftc.teamcode.configurations;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class turelRed {
    private Servo xr;
    private Servo xl;

    private Limelight3A limelight;

    public void init(HardwareMap hwMap){
        limelight = hwMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(7);
        limelight.start();

        xl = hwMap.get(Servo.class, "xl");
        xr = hwMap.get(Servo.class, "xr");

        xl.setPosition(0.5);
        xr.setPosition(0.5);
    }
    public void Turel(){
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()){
            double pos = 0.5 - llResult.getTx() * 0.0115;
            pos = Range.clip(pos, 0.0, 1.0);
            xr.setPosition(pos);
            xl.setPosition(pos);

        }
    }
}
