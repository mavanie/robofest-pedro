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
        Pose startPoseNorth = new Pose( 6.5, 15, Math.toRadians(90));
        //noinspection unused
        Pose startPoseSouth = new Pose(5.5, 15.5, Math.toRadians(-90));
        //noinspection unused
        Pose boxApose = new Pose (8,12, Math.toRadians(-90));
        //noinspection unused
        Pose boxBpose = new Pose (20, 12, Math.toRadians(-90));
        //noinspection unused
        Pose boxCpose = new Pose(31.5, 12, Math.toRadians(-90));
        //noinspection unused
        Pose boxDpose = new Pose (43,12, Math.toRadians(-90));
        //noinspection unused
        Pose boxEpose = new Pose (66,12, Math.toRadians(-90));

        //noinspection unused
        Pose white1Pose = new Pose(13.5, 17.5, Math.toRadians(90));
        //noinspection unused
        Pose white2Pose = new Pose(25.5, 17.5, Math.toRadians(90));
        Pose blackDropPose = new Pose(47,23, Math.toRadians(180));
        Pose medalDropPose = new Pose(50,20, Math.toRadians(180));

        Pose crossPose = new Pose(55, 20, Math.toRadians(0));
        Pose legoSouth = new Pose(55, 11, Math.toRadians(-90));
        Pose legoEast = new Pose(62, 16, Math.toRadians(0));
        Pose legoNorth = new Pose(55, 19, Math.toRadians(90));
        Pose medalPose = new Pose(66, 15.7, Math.toRadians(0));

        // ==================================
        // CHANGE THIS STUFF!
        // ==================================

        //noinspection UnnecessaryLocalVariable
        Pose startPose = startPoseWest;
        //noinspection UnnecessaryLocalVariable
        Pose whitePose = white2Pose;
        Pose stackPose = new Pose(boxCpose.getX(), boxCpose.getY()+0.5, boxCpose.getHeading());
        Pose blackPose = new Pose(boxBpose.getX(), boxBpose.getY()-1, boxBpose.getHeading());

        follower.setStartingPose(startPose);

        PathChain whiteBox = follower.pathBuilder()
            .addPath(new BezierLine(new Point(startPose), new Point(whitePose.getX(), whitePose.getY()-2)))
            .setLinearHeadingInterpolation(startPose.getHeading(),whitePose.getHeading())
            .addPath(new BezierLine(new Point(whitePose.getX(), whitePose.getY()-2), new Point(whitePose)))
            .setPathEndTimeoutConstraint(0)
            .build();
        PathChain stack = follower.pathBuilder()
            .addPath(new BezierLine(new Point(whitePose), new Point (stackPose.getX(), stackPose.getY()+3)))
            .setConstantHeadingInterpolation(stackPose.getHeading())
            .addPath(new BezierLine(new Point(stackPose.getX(), stackPose.getY()+3), new Point(stackPose)))
            .setPathEndTimeoutConstraint(0)
            .build();
        PathChain blackBox = follower.pathBuilder()
            .addPath(new BezierLine(new Point(stackPose), new Point(stackPose.getX(), stackPose.getY()+4)))
            .setConstantHeadingInterpolation(stackPose.getHeading())
            .addPath(new BezierLine(new Point(stackPose.getX(),stackPose.getY()+4), new Point(blackPose.getX(), blackPose.getY()+2)))
            .setConstantHeadingInterpolation(blackPose.getHeading())
            .addPath(new BezierLine(new Point(blackPose.getX(), blackPose.getY()+2), new Point(blackPose)))
            .setConstantHeadingInterpolation(blackPose.getHeading())
            .setPathEndTimeoutConstraint(0)
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
            .addPath(new BezierLine(new Point(legoEast), new Point(crossPose.getX(), crossPose.getY()-5)))
            .setLinearHeadingInterpolation(legoEast.getHeading(), legoNorth.getHeading())
            .addPath(new BezierLine(new Point(crossPose.getX(), crossPose.getY()-5), new Point(legoNorth)))
            .setConstantHeadingInterpolation(legoNorth.getHeading())
            .addPath(new BezierLine(new Point(legoNorth), new Point(blackDropPose)))
            .setConstantHeadingInterpolation(blackDropPose.getHeading())
            .build();
        PathChain medalPath = follower.pathBuilder()
            .addPath(new BezierLine(new Point(blackDropPose), new Point(blackDropPose.getX()+3, blackDropPose.getY())))
            .setConstantHeadingInterpolation(blackDropPose.getHeading())
            .addPath(new BezierLine(new Point(blackDropPose.getX()+3, blackDropPose.getY()), new Point(medalPose)))
            .setConstantHeadingInterpolation(medalPose.getHeading())
            .setPathEndTimeoutConstraint(0)
            .build();
        PathChain medalDropPath = follower.pathBuilder()
            .addPath(new BezierLine(new Point(medalPose), new Point(medalPose.getX()-4, medalPose.getY())))
            .setConstantHeadingInterpolation(medalPose.getHeading())
            .addPath(new BezierLine(new Point(medalPose.getX()-4, medalPose.getY()), new Point(medalDropPose)))
            .setConstantHeadingInterpolation(medalDropPose.getHeading())
            .setPathEndTimeoutConstraint(0)
            .build();
        PathChain endGame = follower.pathBuilder()
            .addPath(new BezierLine(new Point(medalDropPose), new Point(medalDropPose.getX()+4, medalDropPose.getY())))
            .setConstantHeadingInterpolation(medalDropPose.getHeading())
            .addPath(new BezierLine(new Point(medalDropPose.getX()+4, medalDropPose.getY()), new Point(crossPose)))
            .setConstantHeadingInterpolation(medalDropPose.getHeading())
            .addParametricCallback(1, this::liftStart)
            .addPath(new BezierLine(new Point(crossPose), new Point(medalPose.getX()+1, medalPose.getY())))
            .setConstantHeadingInterpolation(0)
            .setPathEndTimeoutConstraint(0)
            .build();

        changeState(0);

        while(!isStopRequested()) {
            follower.update();

            boolean pressed = button.isPressed();
            if (pressed && !oldPressed && stateTime.getElapsedTimeSeconds() > 0.2) {
                if (state == 0) {
                    display.writeNumber(3);
//                        display.writeCharacter('0', 0, true);
//                        display.writeCharacter('1', 1, false);
//                        display.writeCharacter('2', 2, false);
//                        display.writeCharacter('3', 3, false);
//                        display.updateDisplay(); // don't forget to call updateDisplay() or maybe do it automatically
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
                        display.writeCharacter(' ', 0, false);
                        display.writeCharacter(' ', 1, false);
                        display.writeCharacter('G', 2, false);
                        display.writeCharacter('O', 3, false);
                        display.updateDisplay();
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
                    } else if (!follower.isBusy()) {
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
                    } else if (!follower.isBusy()) {
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
                    } else if (!follower.isBusy()) {
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
                    } else if (!follower.isBusy()) {
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
//        display.writeNumber(newState);
        display.updateDisplay();
        oldState = state;
        state = newState;
    }
}
