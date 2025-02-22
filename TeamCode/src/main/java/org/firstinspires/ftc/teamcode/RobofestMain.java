package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@TeleOp
@Config
public class RobofestMain extends LinearOpMode {
    private Follower follower;
    public static Pose startPose = new Pose(28, 7, Math.toRadians(0));
    public static Pose crossPose = new Pose(28, 100, Math.toRadians(0));
    private Servo claw;
    private Servo lift;
    public static double LIFT_DOWN = 0;
    public static double LIFT_UP = 0.6;
    public static double CLAW_OPEN = 0;
    public static double CLAW_CLOSED = 1;
    @Override
    public void runOpMode() throws InterruptedException {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);

        claw = hardwareMap.get(Servo.class, "claw");
        lift = hardwareMap.get(Servo.class, "lift");

        TouchSensor button = hardwareMap.get(TouchSensor.class, "button");
        int state = 0;

        boolean oldPressed = false;

        Path drive = new Path(new BezierLine(new Point(startPose), new Point(crossPose)));

        while(!isStopRequested()) {
            follower.update();
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
                    follower.followPath(drive);
                    state = 2;
                    break;
                case 2:
                    if (pressed && !oldPressed) {
                        state = 0;
                    }
                    break;
            }
            oldPressed = pressed;
        }
    }
}
