package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp(name = "intake", group = "examples")
public class intake extends LinearOpMode {
    private Servo IntakeKoteru1;
    private Servo IntakeKoteru2;
    private Servo Intake;
    private Servo IntakeKozgalu;



    @Override
    public void runOpMode(){
        IntakeKoteru1 = hardwareMap.get(Servo.class, "IntakeKoteru1");
        IntakeKoteru2 = hardwareMap.get(Servo.class, "IntakeKoteru2");
        Intake = hardwareMap.get(Servo.class, "Intake");
        IntakeKozgalu = hardwareMap.get(Servo.class, "IntakeKozgalu");



        //#########################################################################
        IntakeKoteru1.setDirection(Servo.Direction.FORWARD);
        IntakeKoteru2.setDirection(Servo.Direction.REVERSE);


        waitForStart();
        if (opModeIsActive()){
            while (opModeIsActive()){
                Intake();

                telemetry.update();
            }
        }

    }

    private void Intake() {
        if (gamepad2.b) {
            Intake.setPosition(0);
            sleep(300);
            IntakeKoteru1.setPosition(1);
            IntakeKoteru2.setPosition(1);
        }
        if (gamepad2.right_bumper) {
            Intake.setPosition(1);
            sleep(400);
            IntakeKoteru1.setPosition(0.2);
            IntakeKoteru2.setPosition(0.2);
        } if (gamepad2.a){
            Intake.setPosition(1);
            IntakeKoteru1.setPosition(0);
            IntakeKoteru2.setPosition(0);
        }
        double IntakeKozgaluu = -gamepad2.left_stick_x;
        IntakeKozgalu.setPosition((IntakeKozgaluu)+0.5);
    }

}
