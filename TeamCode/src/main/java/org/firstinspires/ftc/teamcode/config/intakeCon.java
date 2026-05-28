package org.firstinspires.ftc.teamcode.config;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class intakeCon  {

    private DcMotor intake, turel;
    boolean tfIn = false;
    boolean tfIn2 = true;

    public void init(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, "intake");
        turel = hardwareMap.get(DcMotor.class, "turel");
    }


    public void intake(boolean rb) {
        if (rb && !tfIn){
            tfIn2 = false;
            if (intake.getPower() == 0){
                tfIn = true;
                tfIn2 = true;
                intake.setPower(1);
            }else if(intake.getPower() > 0){
                tfIn = true;
                tfIn2 = true;
                intake.setPower(0);
            }
        }
        else if(!rb && tfIn2){
            tfIn = false;
        }
    }
}
