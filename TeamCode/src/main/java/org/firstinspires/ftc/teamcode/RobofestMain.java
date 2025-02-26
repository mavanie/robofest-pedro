package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
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

    public static Pose white1Pose = new Pose(13, 16, Math.toRadians(90));
    public static Pose white2Pose = new Pose(25.5, 16, Math.toRadians(90));
    public static Pose whitePose = white1Pose;
    public static Pose crossPose = new Pose(55, 14, Math.toRadians(0));
    public static Pose boxPose = new Pose(30, 14, Math.toRadians(0));
    public static Pose pickupPose = new Pose(30, 9, Math.toRadians(-90));
    private Servo claw;
    private Servo lift;
    private AlphaDisplay display;
    public static double LIFT_DOWN = 0;
    public static double LIFT_UP = 0.6;
    public static double CLAW_OPEN = 1;
    public static double CLAW_CLOSED = 0;
    private Timer stateTime = new Timer();
    private int state = 0;
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

        PathChain box = follower.pathBuilder()
            .addPath(new BezierLine(new Point(startPose), new Point(boxPose)))
            .build();
        PathChain pickup = follower.pathBuilder()
            .addPath(new BezierLine(new Point(boxPose), new Point(pickupPose)))
            .build();
        PathChain whiteBox = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(whitePose)))
                .setLinearHeadingInterpolation(startPose.getHeading(),whitePose.getHeading())
                .build();

        while(!isStopRequested()) {
            follower.update();
            boolean pressed = button.isPressed();
            if (pressed && !oldPressed) {
                if (state == 0) {
                    follower.followPath(whiteBox);
                   changeState(1);
                }else{
                    changeState(0);
                }
            }
            switch (state) {
                case 0:
                    follower.breakFollowing();
                    follower.setPose(startPose);
                    if (gamepad1.a) {
                        liftDown();
                    }
                    if (gamepad1.b) {
                        liftUp();
                    }
                    if (gamepad1.x) {
                        claw.setPosition(CLAW_OPEN);
                    }
                    if (gamepad1.y) {
                        closeClaw();
                    }
                    break;
                case 1:
                    if (!follower.isBusy()) {
                        liftDown();
                        changeState(2);
                    }
                    break;
                case 2:
                    if (stateTime.getElapsedTimeSeconds() > 4.6)  {
                        closeClaw();
                        changeState(3);
                    }
                    break;
                case 3:
                    if (stateTime.getElapsedTimeSeconds() > 1.4)  {
                        liftUp();
                        changeState(4);
                    }
                    break;
            }
            oldPressed = pressed;
            telemetry.addData("state", state);
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

    private void liftUp() {
        lift.setPosition(LIFT_UP);
    }

    private void closeClaw() {
        claw.setPosition(CLAW_CLOSED);
    }

    private void liftDown() {
        lift.setPosition(LIFT_DOWN);
    }
    private void changeState(int newState) {
        stateTime.resetTimer();
        display.writeNumber(newState);
        state = newState;

    }
}
