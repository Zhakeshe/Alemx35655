package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
public class vib extends OpMode {

    @Override
    public void init() {

    }

    @Override
    public void loop() {
        if (gamepad1.a){
            gamepad2.rumble(1.0, 1.0, 8000);
            gamepad1.rumble(1.0, 1.0, 8000);
        }else {
            gamepad1.stopRumble();
            gamepad2.stopRumble();
        }
    }
}
