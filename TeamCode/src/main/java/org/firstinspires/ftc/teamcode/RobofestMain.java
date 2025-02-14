package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp
@Config
public class RobofestMain extends LinearOpMode {
    private Servo claw;
    private Servo lift;
    public static double LIFT_DOWN = 0;
    public static double LIFT_UP = 0.6;
    public static double CLAW_OPEN = 0;
    public static double CLAW_CLOSED = 1;
    @Override
    public void runOpMode() throws InterruptedException {
        claw = hardwareMap.get(Servo.class, "claw");
        lift = hardwareMap.get(Servo.class, "lift");
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
                    if (gamepad1.a) {
                        lift.setPosition(LIFT_DOWN);
                    }
                    if (gamepad1.b) {
                        lift.setPosition(LIFT_UP);
                    }
                    if (gamepad1.x) {
                        claw.setPosition(CLAW_OPEN);
                    }
                    if (gamepad1.y) {
                        claw.setPosition(CLAW_CLOSED);
                    }
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
