package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
@Config
@TeleOp
public class MotorTest extends LinearOpMode {
public static double POSITION_A = 0.6;
public static double POSITION_B = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        Servo servo = hardwareMap.get(Servo.class, "lift");
        while(!isStopRequested()) {
            if (gamepad1.a) {
                servo.setPosition(POSITION_A);
            }
            if (gamepad1.b) {
                servo.setPosition(POSITION_B);
            }
        }
    }
}
