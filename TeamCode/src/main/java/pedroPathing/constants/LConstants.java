package pedroPathing.constants;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
@Config
public class LConstants {
    static {
        TwoWheelConstants.forwardTicksToInches = 0.00200; // default 0.001989436789
        TwoWheelConstants.strafeTicksToInches = 0.00200;
        TwoWheelConstants.forwardY = -89;
        TwoWheelConstants.strafeX = 11;
        TwoWheelConstants.forwardEncoder_HardwareMapName = "frontLeft";
        TwoWheelConstants.strafeEncoder_HardwareMapName = "backRight";
        TwoWheelConstants.forwardEncoderDirection = Encoder.REVERSE;
        TwoWheelConstants.strafeEncoderDirection = Encoder.FORWARD;
        TwoWheelConstants.IMU_HardwareMapName = "imu";
        TwoWheelConstants.IMU_Orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
    }
}




