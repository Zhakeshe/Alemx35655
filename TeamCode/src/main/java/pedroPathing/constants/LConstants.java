package pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class LConstants {
    static {
        ThreeWheelConstants.forwardTicksToInches = 0.002;
        ThreeWheelConstants.strafeTicksToInches = 0.002;
        ThreeWheelConstants.turnTicksToInches = 0.0038;
        ThreeWheelConstants.leftY = 12;
        ThreeWheelConstants.rightY = -12;
        ThreeWheelConstants.strafeX = -3;
        ThreeWheelConstants.leftEncoder_HardwareMapName = "leftRear";
        ThreeWheelConstants.rightEncoder_HardwareMapName = "rightRear";
        ThreeWheelConstants.strafeEncoder_HardwareMapName = "rightFront";
        ThreeWheelConstants.leftEncoderDirection = Encoder.REVERSE;
        ThreeWheelConstants.rightEncoderDirection = Encoder.REVERSE;
        ThreeWheelConstants.strafeEncoderDirection = Encoder.FORWARD;
    }
}




