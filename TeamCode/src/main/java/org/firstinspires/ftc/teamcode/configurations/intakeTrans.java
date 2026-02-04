package org.firstinspires.ftc.teamcode.configurations;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class intakeTrans {
    private DcMotor Intake,Trans;


    boolean tfin1 = false;
    boolean tfin2 = false;

    boolean tftrans1 = false;
    boolean tftrans2 = false;

    public void init(HardwareMap hwMap){
        Intake = hwMap.get(DcMotor.class, "Intake");
        Trans = hwMap.get(DcMotor.class, "Trans");

        Intake.setDirection(DcMotorSimple.Direction.REVERSE);
        Trans.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void Intake(boolean g1tsb, boolean g1a, Gamepad gamepad1){
        if(g1tsb && !tfin1){
            tfin2 = false;
            if(Intake.getPower() == 0){
                tfin1 = true;

                tfin2 = true;
                Intake.setPower(1);
                Trans.setPower(-1);

            } else if (Intake.getPower() > 0) {
                tfin1 = true;
                tfin2 = true;
                Intake.setPower(0);
                Trans.setPower(0);

            }
        }
        else if(!g1tsb && tfin2){
            tfin1 = false;
        }

        if (g1a && !tftrans1){
            tftrans2 = false;
            if (Trans.getPower() > 0){
                tftrans1 = true;
                tftrans2 = true;
                Trans.setPower(-1);
            } else if (Trans.getPower() < 0) {
                tftrans1 = true;
                tftrans2 = true;
                Trans.setPower(1);
            }
        }
        else if (!g1a && tftrans2) {
            tftrans1 = false;
        }

        if (Intake.getPower() > 0){
            gamepad1.rumble(0.0, 0.5, 1000000000);
        }else {
            gamepad1.stopRumble();
        }
    }

}
