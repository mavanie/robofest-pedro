package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp
public class RobofestMain extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        DcMotorEx backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        TouchSensor button = hardwareMap.get(TouchSensor.class, "button");
        int state = 0;

        boolean oldPressed = false;


        while(!isStopRequested()) {
            boolean pressed = button.isPressed();
            switch (state) {
                case 0:
                    if (pressed && !oldPressed) {
                        state = 1;
                    }
                    break;
                case 1:
                    frontLeft.setPower(1);
                    frontRight.setPower(1);
                    backLeft.setPower(1);
                    backRight.setPower(1);
                    if (pressed && !oldPressed) {
                        frontLeft.setPower(0);
                        frontRight.setPower(0);
                        backLeft.setPower(0);
                        backRight.setPower(0);
                        state = 0;
                    }
                    break;
            }
            oldPressed = pressed;
        }
    }
}
