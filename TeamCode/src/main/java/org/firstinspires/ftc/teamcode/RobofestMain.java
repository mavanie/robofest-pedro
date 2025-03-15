package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.util.Timer;
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
    public static Pose startPoseEast = new Pose(5.5, 14, Math.toRadians(0));
    public static Pose startPoseWest = new Pose(5.5, 14, Math.toRadians(180));
    public static Pose startPoseNorth = new Pose(5.5, 14, Math.toRadians(90));
    public static Pose startPoseSouth = new Pose(5.5, 14, Math.toRadians(-90));
    public static Pose startPose = startPoseEast;

    public static Pose boxBpose = new Pose (18.5, 13, Math.toRadians(-90));
    public static Pose boxCpose = new Pose(30, 13, Math.toRadians(-90));
    public static Pose stackPose = boxBpose;
    public static Pose blackPose = boxCpose;
    public static Pose white1Pose = new Pose(13.5, 16.5, Math.toRadians(90));
    public static Pose white2Pose = new Pose(25.5, 16.5, Math.toRadians(90));
    public static Pose whitePose = white1Pose;
    public static Pose crossPose = new Pose(55, 14, Math.toRadians(0));
    public static Pose pickupPose = new Pose(30, 9, Math.toRadians(-90));
    private Servo claw;
    private Servo lift;
    private AlphaDisplay display;
    public static double LIFT_DOWN = 0;
    public static double LIFT_UP = 0.6;
    public static double LIFT_MEDAL = 0.3;
    public static double CLAW_OPEN = 1;
    public static double CLAW_CLOSED = 0;
    private Timer stateTime = new Timer();
    private int state = -1;
    private int oldState = -1;
    @Override
    public void runOpMode(){
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        claw = hardwareMap.get(Servo.class, "claw");
        lift = hardwareMap.get(Servo.class, "lift");
        display = hardwareMap.get(AlphaDisplay.class, "display");

        TouchSensor button = hardwareMap.get(TouchSensor.class, "button");
        boolean oldPressed = false;

//        PathChain box = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(startPose), new Point(boxPose)))
//                .build();
//        PathChain pickup = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(boxPose), new Point(pickupPose)))
//                .build();
        PathChain whiteBox = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(whitePose)))
                .setLinearHeadingInterpolation(startPose.getHeading(),whitePose.getHeading())
                .build();
        PathChain stack = follower.pathBuilder()
                .addPath(new BezierLine(new Point(whitePose), new Point (stackPose)))
                .setLinearHeadingInterpolation(whitePose.getHeading(), stackPose.getHeading())
                .build();
        PathChain back = follower.pathBuilder()
                .addPath(new BezierLine(new Point(stackPose), new Point(stackPose.getX(), stackPose.getY() + 5)))
                .setConstantHeadingInterpolation(stackPose.getHeading())
                .build();


        changeState(0);

        while(!isStopRequested()) {
            follower.update();

            boolean enter = state != oldState;
            oldState = state;

            boolean pressed = button.isPressed();
            if (pressed && !oldPressed) {
                if (state == 0) {
                    changeState(1);
                } else {
                    changeState(0);
                }
            }

            switch (state) {
                case 0:
                    if(enter) {
                        liftUp();
                        openClaw();
                    }
                    follower.breakFollowing();
                    follower.setPose(startPose);
                    if (gamepad1.dpad_down) {
                        liftDown();
                    }
                    if (gamepad1.dpad_left) {
                        liftMedal();
                    }
                    if (gamepad1.dpad_up) {
                        liftUp();
                    }
                    if (gamepad1.x) {
                        openClaw();
                    }
                    if (gamepad1.y) {
                        closeClaw();
                    }
                    break;
                case 1:
                    if (enter) {
                        follower.followPath(whiteBox);
                    }
                    if (!follower.isBusy()) {
                        changeState(2);
                    }
                    break;
                case 2:
                    if (enter) {
                        liftDown();
                    }
                    if (stateTime.getElapsedTimeSeconds() > 4.6) {
                        changeState(3);
                    }
                    break;
                case 3:
                    if (enter) {
                        closeClaw();
                    }
                    if (stateTime.getElapsedTimeSeconds() > 1.4) {
                        changeState(4);
                    }
                    break;
                case 4:
                    if (enter) {
                        liftUp();
                    }
                    if (stateTime.getElapsedTimeSeconds() > 3) {
                        changeState(5);
                    }
                    break;
                case 5:
                    if (enter) {
                        follower.followPath(stack);
                    }
                    if (!follower.isBusy()) {
                        changeState(6);
                    }
                    break;
                case 6:
                    if (stateTime.getElapsedTimeSeconds() > 1) {
                        changeState(7);
                    }
                case 7:
                    if (enter) {
                        openClaw();
                    }
                    if (stateTime.getElapsedTimeSeconds() > 1.4) {
                        changeState(8);
                    }
                    break;
                case 8:
                    if (enter) {
                        follower.followPath(back);
                    }
                    if (!follower.isBusy()) {
                        changeState(9);
                    }
            }
            oldPressed = pressed;
            telemetry.addData("state", state);
            telemetry.addData("stateTime", stateTime.getElapsedTimeSeconds());
            telemetry.addData("busy", follower.isBusy());
            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.addData("Heading Error", Math.toDegrees(follower.headingError));
            telemetry.addData("lift", lift.getPosition());
            telemetry.addData("claw", claw.getPosition());
            telemetry.update();
        }
    }

    private void openClaw() {
        claw.setPosition(CLAW_OPEN);
    }

    private void liftUp() {
        lift.setPosition(LIFT_UP);
    }

    private void liftDown() {
        lift.setPosition(LIFT_DOWN);
    }

    private  void liftMedal() {
        lift.setPosition(LIFT_MEDAL);
    }

    private void closeClaw() {
        claw.setPosition(CLAW_CLOSED);
    }

    private void changeState(int newState) {
        stateTime.resetTimer();
        display.writeNumber(newState);
        display.updateDisplay();
        oldState = state;
        state = newState;
    }
}
