package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@TeleOp(name = "servotest", group = "examples")
public class servotest extends LinearOpMode {
    private Servo Servoqwe;
    private Servo Servoqwe2;

    @Override
    public void runOpMode(){
        Servoqwe = hardwareMap.get(Servo.class, "Servoqwe");
        Servoqwe2 = hardwareMap.get(Servo.class, "Servoqwe2");

        waitForStart();
        Servoqwe.setPosition(0.5);
        if (opModeIsActive()) {

            while (opModeIsActive()) {
                if (gamepad1.a){
                    Servoqwe.setPosition(1);
                    Servoqwe2.setPosition(1);
                } if (gamepad1.b) {
                    Servoqwe.setPosition(0);
                    Servoqwe2.setPosition(0);
                } if (gamepad1.y) {
                    Servoqwe.setPosition(0.8);
                    Servoqwe2.setPosition(0.8);
                }

                telemetry.update();

            }
        }


    }
}
