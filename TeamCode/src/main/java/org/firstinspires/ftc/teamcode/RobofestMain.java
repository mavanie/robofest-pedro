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
    /** @noinspection FieldCanBeLocal*/
    private Follower follower;
    private Servo claw;
    private Servo lift;
    private AlphaDisplay display;
    public static double LIFT_DOWN = 0;
    public static double LIFT_UP = 0.6;
    public static double LIFT_MEDAL = 0.3;
    public static double LIFT_START = 0.4;
    public static double CLAW_OPEN = 1;
    public static double CLAW_CLOSED = 0;
    private final Timer stateTime = new Timer();
    private int state = -1;
    private int oldState = -1;
    @Override
    public void runOpMode(){
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setMaxPower(0.6);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        claw = hardwareMap.get(Servo.class, "claw");
        lift = hardwareMap.get(Servo.class, "lift");
        display = hardwareMap.get(AlphaDisplay.class, "display");

        TouchSensor button = hardwareMap.get(TouchSensor.class, "button");
        boolean oldPressed = false;

        //noinspection unused
        Pose startPoseEast = new Pose(5.5, 14, Math.toRadians(0));
        //noinspection unused
        Pose startPoseWest = new Pose(5.5, 14, Math.toRadians(180));
        //noinspection unused
        Pose startPoseNorth = new Pose(5.5, 14, Math.toRadians(90));
        //noinspection unused
        Pose startPoseSouth = new Pose(5.5, 14, Math.toRadians(-90));
        //noinspection UnnecessaryLocalVariable
        Pose startPose = startPoseEast;
        //noinspection unused
        Pose boxApose = new Pose (8,11.5, Math.toRadians(-90));
        //noinspection unused
        Pose boxBpose = new Pose (19.5, 11.5, Math.toRadians(-90));
        //noinspection unused
        Pose boxCpose = new Pose(31.5, 11.5, Math.toRadians(-90));
        //noinspection unused
        Pose boxDpose = new Pose (43,11.5, Math.toRadians(-90));
        //noinspection unused
        Pose boxEpose = new Pose (66,11.5, Math.toRadians(-90));
        //noinspection UnnecessaryLocalVariable
        Pose stackPose = boxDpose;
        Pose blackPose = new Pose(boxApose.getX(), boxApose.getY()-1, boxApose.getHeading());

        //noinspection unused
        Pose white1Pose = new Pose(13.5, 17.5, Math.toRadians(90));
        //noinspection unused
        Pose white2Pose = new Pose(25.5, 17.5, Math.toRadians(90));
        Pose blackDropPose = new Pose(43,18.5 , Math.toRadians(135));
        Pose medalDropPose = new Pose(42, 17.5, Math.toRadians(135));
        //noinspection UnnecessaryLocalVariable
        Pose whitePose = white1Pose;

        Pose crossPose = new Pose(55, 20, Math.toRadians(0));
        Pose legoSouth = new Pose(55, 11, Math.toRadians(-90));
        Pose legoEast = new Pose(62, 16, Math.toRadians(0));
        Pose legoNorth = new Pose(55, 19, Math.toRadians(90));
        Pose medalPose = new Pose(66, 17, Math.toRadians(0));

        follower.setStartingPose(startPose);

        PathChain whiteBox = follower.pathBuilder()
            .addPath(new BezierLine(new Point(startPose), new Point(whitePose.getX(), whitePose.getY()-2)))
            .setLinearHeadingInterpolation(startPose.getHeading(),whitePose.getHeading())
            .addPath(new BezierLine(new Point(whitePose.getX(), whitePose.getY()-2), new Point(whitePose)))
            .build();
        PathChain stack = follower.pathBuilder()
            .addPath(new BezierLine(new Point(whitePose), new Point (stackPose.getX(), stackPose.getY()+3)))
            .setConstantHeadingInterpolation(stackPose.getHeading())
            .addPath(new BezierLine(new Point(stackPose.getX(), stackPose.getY()+3), new Point(stackPose)))
            .build();
        PathChain blackBox = follower.pathBuilder()
            .addPath(new BezierLine(new Point(stackPose), new Point(stackPose.getX(), stackPose.getY()+4)))
            .setConstantHeadingInterpolation(stackPose.getHeading())
            .addPath(new BezierLine(new Point(stackPose.getX(),stackPose.getY()+4), new Point(blackPose.getX(), blackPose.getY()+2)))
            .setConstantHeadingInterpolation(blackPose.getHeading())
            .addPath(new BezierLine(new Point(blackPose.getX(), blackPose.getY()+2), new Point(blackPose)))
            .build();
        PathChain lego = follower.pathBuilder()
            .addPath(new BezierLine(new Point(blackPose), new Point(blackPose.getX(), blackPose.getY()+8)))
            .setConstantHeadingInterpolation(blackPose.getHeading())
            .addPath(new BezierLine(new Point(blackPose.getX(), blackPose.getY()+8), new Point(crossPose)))
            .setConstantHeadingInterpolation(blackPose.getHeading())
            .addPath(new BezierLine(new Point(crossPose), new Point(legoSouth)))
            .setConstantHeadingInterpolation(legoSouth.getHeading())
            .addPath(new BezierLine(new Point(legoSouth), new Point(crossPose)))
            .setConstantHeadingInterpolation(legoSouth.getHeading())
            .addPath(new BezierLine(new Point(crossPose), new Point(legoEast)))
            .setConstantHeadingInterpolation(legoEast.getHeading())
            .addPath(new BezierLine(new Point(legoEast), new Point(legoNorth)))
            .setConstantHeadingInterpolation(legoNorth.getHeading())
            .addPath(new BezierLine(new Point(legoNorth), new Point(blackDropPose)))
            .setConstantHeadingInterpolation(blackDropPose.getHeading())
            .build();
        PathChain medalPath = follower.pathBuilder()
            .addPath(new BezierLine(new Point(blackDropPose), new Point(blackDropPose.getX()+3, blackDropPose.getY()-3)))
            .setConstantHeadingInterpolation(blackDropPose.getHeading())
            .addPath(new BezierLine(new Point(blackDropPose.getX()+3, blackDropPose.getY()-3), new Point(medalPose)))
            .setConstantHeadingInterpolation(medalPose.getHeading())
            .build();
        PathChain medalDropPath = follower.pathBuilder()
            .addPath(new BezierLine(new Point(medalPose), new Point(medalPose.getX()-4, medalPose.getY())))
            .setConstantHeadingInterpolation(medalPose.getHeading())
            .addPath(new BezierLine(new Point(medalPose.getX()-4, medalPose.getY()), new Point(medalDropPose)))
            .setConstantHeadingInterpolation(medalDropPose.getHeading())
            .build();
        PathChain endGame = follower.pathBuilder()
            .addPath(new BezierLine(new Point(medalDropPose), new Point(medalDropPose.getX()+2, medalDropPose.getY()-2)))
            .setConstantHeadingInterpolation(medalDropPose.getHeading())
            .addPath(new BezierLine(new Point(medalDropPose.getX()+2, medalDropPose.getY()-2), new Point(crossPose)))
            .build();

        changeState(0);

        while(!isStopRequested()) {
            follower.update();

            boolean pressed = button.isPressed();
            if (pressed && !oldPressed && stateTime.getElapsedTimeSeconds() > 0.2) {
                if (state == 0) {
                    changeState(10);
                } else {
                    changeState(0);
                }
            }

            boolean enter = state != oldState;
            oldState = state;

            switch (state) {
                case 0:
                    if(enter) {
                        liftStart();
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
                case 10:
                    if (enter) {
                        follower.followPath(whiteBox);
                    } else if (follower.atParametricEnd()) {
                        follower.breakFollowing();
                        changeState(20);
                    }
                    break;
                case 20:
                    if (enter) {
                        liftDown();
                    } else if (stateTime.getElapsedTimeSeconds() > 0.85) {
                        changeState(30);
                    }
                    break;
                case 30:
                    if (enter) {
                        closeClaw();
                    } else if (stateTime.getElapsedTimeSeconds() > 1.4) {
                        changeState(40);
                    }
                    break;
                case 40:
                    if (enter) {
                        liftUp();
                    } else if (stateTime.getElapsedTimeSeconds() > 1) {
                        changeState(50);
                    }
                    break;
                case 50:
                    if (enter) {
                        follower.followPath(stack);
                    } else if (!follower.isBusy()) {
                        changeState(60);
                    }
                    break;
                case 60:
                    if (stateTime.getElapsedTimeSeconds() > 0.4) {
                        changeState(70);
                    }
                    break;
                case 70:
                    if (enter) {
                        openClaw();
                    }else if (stateTime.getElapsedTimeSeconds() > 1.4) {
                        changeState(90);
                    }
                    break;
                case 90:
                    if (enter) {
                        follower.followPath(blackBox);
                    } else if (follower.atParametricEnd()) {
                        follower.breakFollowing();
                        changeState(100);
                    }
                    break;
                case 100:
                    if (enter) {
                        liftDown();
                    } else if (stateTime.getElapsedTimeSeconds() > 1) {
                        changeState(110);
                    }
                    break;
                case 110:
                    if (enter) {
                        closeClaw();
                    } else if (stateTime.getElapsedTimeSeconds() > 1.4) {
                        changeState(120);
                    }
                    break;
                case 120:
                    if (enter) {
                        follower.followPath(lego);
                    } else if (!follower.isBusy()) {
                        changeState(140);
                    }
                    break;
                case 140:
                    if (enter) {
                        openClaw();
                    } else if (stateTime.getElapsedTimeSeconds() > 1.4) {
                        changeState(150);
                    }
                    break;
                case 150:
                    if (enter) {
                        follower.followPath(medalPath);
                    } else if (!follower.isBusy()) {
                        changeState(160);
                    }
                    break;
                case 160:
                    if (enter) {
                        liftMedal();
                    } else if (stateTime.getElapsedTimeSeconds() > 0.7) {
                        changeState(170);
                    }
                    break;
                case 170:
                    if (enter) {
                       follower.followPath(medalDropPath);
                    } else if (follower.atParametricEnd()) {
                        changeState(180);
                    }
                    break;
                case 180:
                    if (enter) {
                        liftDown();
                    } else if (stateTime.getElapsedTimeSeconds() > 0.7) {
                        changeState(190);
                    }
                    break;
                case 190:
                    if (enter) {
                        follower.followPath(endGame);
                    } else if (follower.atParametricEnd()) {
                        changeState(200);
                    }
                    break;
                case 200:
                    if (enter) {
                        follower.breakFollowing();
                    }
                    break;
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

    private void liftStart() {
        lift.setPosition(LIFT_START);
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
