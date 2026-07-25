package org.firstinspires.ftc.teamcode.pedroPathing.WorkingCode.BLUE;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.HashMap;

//@Autonomous
public class BLUEGoalTouchWithLower extends OpMode {
    //variables + lists
    HashMap<String, Integer> indices = new HashMap<String, Integer>();
    private int numBall = 0;
    private static final double gatePosdown = 0.6;
    private static final double gatePosUp = 0.2;
    private int LastAprilTag;
    double distanceINCH;

    //declaring motors, servos, + cameras
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private Rev2mDistanceSensor distance;
    private DcMotor intake;
    private DcMotor chooChoo;
    private Servo gate;
    private DigitalChannel touchSensor;

    //timers
    private ElapsedTime  runtime = new ElapsedTime();
    private Timer intakeTimer2;
    private Timer pathTimer;
    private Timer launchTimer2;
    private Timer aprilTimer;
    private Timer aprilIntakeTimer;
    private double intakeTimer;

    //positions for follower
    private Follower follower;
    private int pathState;
    private final Pose startPose = new Pose(25.762, 128.207, Math.toRadians(135)); // Start Pose of our robot.
    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path4;
    public PathChain PathPark;


    public void buildPaths() { //sets up all the paths that the robot will follow
        //moves forward to launching position
        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(25.762, 128.207),

                                new Pose(54.209480961923845, 98.61067134268535)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(44))

                .build();

        //moves to pick up more + rotate 180
        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(54.209480961923845, 98.61067134268535),

                                new Pose(49.094, 86.459)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(44), Math.toRadians(180))

                .build();

        //picks up artifacts
        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(49.094, 86.459),

                                new Pose(14, 86.459) //used to be 19.035164328657316
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        //returns to launching position
        Path4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(14, 86.459),

                                new Pose(54.209480961923845, 98.61067134268535)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(44))

                .build();

        //parks to face 90 degrees
        PathPark = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(54.209480961923845, 98.61067134268535),

                                new Pose(50, 60)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(44), Math.toRadians(90))

                .build();
    }

    public void LaunchSequence(int state){
        if (indices.get("launchSequence") == 0) { //resets timer
            telemetry.addData("chooChoo", 0);
            runtime.reset();
            indices.put("launchSequence",1);
        } else if (indices.get("launchSequence") == 1 && runtime.seconds() > .5) { //waits 0.5 seconds, then runs chooChoo
            runtime.reset();
            chooChoo.setPower(1.0);
            telemetry.addData("chooChoo", 1);
            indices.put("launchSequence",2);
        } else if (indices.get("launchSequence") == 2 && runtime.seconds() >= 1.0) { //stops chooChoo after 1 second
            chooChoo.setPower(0);
            telemetry.addData("chooChoo", 2);
            launchTimer2.resetTimer();
            indices.put("launchSequence",3);
        } else if (indices.get("launchSequence") == 3){ //runs chooChoo until it is lowered all the way, then proceeds to next step
            telemetry.addData("chooChoo", 3);
            chooChoo.setPower(1);
            if ((distanceINCH <= 3 && distanceINCH > 0 && Double.isFinite(distanceINCH)) || launchTimer2.getElapsedTimeSeconds() >= 0.5 ) { //used to be 2
                chooChoo.setPower(0);
                telemetry.addData("chooChoo", 4);
                indices.put("launchSequence",0);
                indices.put("launchIntake",state);
            }
        }
    }

    //intake sequence
    public void IntakeSequence(int state, double timer) {
        if (indices.get("intakeSequence") == 0) { //runs intake
            telemetry.addData("intake", 0);
            intakeTimer2.resetTimer();
            intake.setPower(1.0);
            numBall++;
            gate.setPosition(gatePosdown);
            indices.put("intakeSequence",1);
        } else if (indices.get("intakeSequence") == 1) {
            if (numBall == 2) { //if it is on its last ball, it will wait 0.8 seconds + run intake
                if (!touchSensor.getState()) {
                    telemetry.addData("intake", 67);
                    intake.setPower(-0.25);
                    gate.setPosition(gatePosUp); //opens the gate more to let out artifact //used to be 0
                    intakeTimer2.resetTimer();
                    numBall = 3;
                }
            } else if (numBall == 3) { //then it will wait 1.3 seconds and close the gate
                if (intakeTimer2.getElapsedTimeSeconds() >= 0.1) {
                    telemetry.addData("intake", 1);
                    indices.put("intakeSequence",3);
                }
            } else if (numBall < 2){ //if it is on its first ball it will raise it for 0.8 seconds
                if (!touchSensor.getState()|| intakeTimer2.getElapsedTimeSeconds() >= 0.8) { //used to be 0.8
                    intakeTimer2.resetTimer();
                    gate.setPosition(gatePosUp);
                    intake.setPower(-0.25);
                    indices.put("intakeSequence",2);
                }
            }
        } else if (indices.get("intakeSequence") == 2){
            if (intakeTimer2.getElapsedTimeSeconds() >= 0.1){
                telemetry.addData("intake", 1);
                indices.put("intakeSequence",3);
            }
        }
        else if (indices.get("intakeSequence") == 3){
            telemetry.addData("intake", 2);
            intake.setPower(0);
            gate.setPosition(gatePosdown);
            indices.put("intakeSequence",0);
            indices.put("launchIntake",state); //proceed to next step in main program
        }
    }


    public void Paths (PathChain path, int state){
        if (indices.get("paths") == 0){ //follows specified path
            follower.followPath(path);
            indices.put("paths",1);
        } else if (indices.get("paths") == 1){ //next step once path is done
            if (!follower.isBusy()){
                indices.put("paths",2);
                pathTimer.resetTimer();
            }
        } else if (indices.get("paths") == 2){ //next step once path is done
            if (pathTimer.getElapsedTimeSeconds() >= 1){ //waits 1 second
                pathState = state;
                indices.put("paths",0);
            }
        }
    }

    public void AprilPaths(PathChain path, int state, boolean intaking) {
        if (indices.get("aprilPaths") == 0) {//runs path first
            aprilIntakeTimer.resetTimer();
            indices.put("aprilPaths", 1);
        } else if (indices.get("aprilPaths") == 1) {
            if (intaking) {
                follower.followPath(path, .35, true);
                intake.setPower(1.0);
                aprilTimer.resetTimer();
            } else {
                follower.followPath(path);
            }
            indices.put("aprilPaths", 3);
        } else if (indices.get("aprilPaths") == 2) {
            if (intaking) {
                if (!touchSensor.getState() || intakeTimer2.getElapsedTimeSeconds() >= 1){ //used to be 1, then 2
                    intake.setPower(-0.25);
                    gate.setPosition(gatePosUp);
                    aprilIntakeTimer.resetTimer();
                    aprilTimer.resetTimer();
                    indices.put("aprilPaths", 3);
                }
            }  //used to be else go to step 3
        }
        else if (indices.get("aprilPaths") == 3) { //stops path after path is done, next step
            if (intaking){
                if (aprilIntakeTimer.getElapsedTimeSeconds() >= 0.1){ //used to be 0.1
                    gate.setPosition(gatePosdown);
                }
            }
            if (!follower.isBusy() && aprilTimer.getElapsedTimeSeconds() >= 0.5) {
                indices.put("aprilPaths",0);
                indices.put("pathSequence", state);
            }
        }
    }

    public void PathSequence(PathChain path1, PathChain path2, PathChain path3, int state){
        if (indices.get("pathSequence") == 0){
            numBall = 0;
            telemetry.addData("path", 1);
            AprilPaths(path1, 1, false);
        } else if (indices.get("pathSequence") == 1){
            telemetry.addData("path", 2);
            AprilPaths(path2, 2, true);
        } else if (indices.get("pathSequence") == 2){
            intake.setPower(0);
            telemetry.addData("path", 3);
            AprilPaths(path3, 3, false);
        } else if (indices.get("pathSequence") == 3){
            telemetry.addData("path", 4);
            indices.put("pathSequence", 0);
            pathState = state;
        }
    }

    public void LaunchIntake(int state){
        if (indices.get("launchIntake") == 0) { //first launch
            telemetry.addData("launchSequence", 0);
            LaunchSequence(1);
        } else if (indices.get("launchIntake") == 1) { //first intake
            telemetry.addData("launchSequence", 1);
            IntakeSequence(2, 0.4);
        } else if (indices.get("launchIntake") == 2){ //second launch
            telemetry.addData("launchSequence", 2);
            LaunchSequence(3);
        } else if (indices.get("launchIntake") == 3){ //second intake
            telemetry.addData("launchSequence", 3);
            IntakeSequence(4, 0.2);
        } else if (indices.get("launchIntake") == 4){ //third launch
            telemetry.addData("launchSequence", 4);
            LaunchSequence(5);
        } else if (indices.get("launchIntake") == 5){
            telemetry.addData("launchSequence", 5);
            indices.put("launchIntake",0);
            pathState = state;
        }
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: //forward to launching position
                telemetry.addData("state", 0);
                Paths(Path1,1);
                break;
            case 1: //cycle of launching 3 artifacts
                telemetry.addData("state", 1);
                LaunchIntake(2);
                break;
            case 2: //follows path to pick up more artifacts
                PathSequence(Path2, Path3, Path4, 3);
                break;
            case 3: //2nd cycle of launching 3 artifacts
                LaunchIntake(4);
                break;
            case 4: //path to park at end
                Paths(PathPark, 5);
                break;
            default:
                break;
        }
    }

    @Override
    public void loop() {
        distanceINCH = distance.getDistance(DistanceUnit.INCH); //used to be line 320
        follower.update();
        autonomousPathUpdate();
        telemetry.addData("timer intake", intakeTimer2.getElapsedTimeSeconds());
        telemetry.addData("timer launch", launchTimer2.getElapsedTimeSeconds());
        telemetry.addData("gateposition", gate.getPosition());



        telemetry.update();
    }

    @Override
    public void init() {
        LastAprilTag = 1;
        intakeTimer2 = new Timer();
        launchTimer2 = new Timer();
        pathTimer = new Timer();
        aprilTimer = new Timer();
        aprilIntakeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        chooChoo = hardwareMap.get(DcMotor.class, "chooChoo");
        chooChoo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        chooChoo.setPower(0);

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setPower(0);

        gate = hardwareMap.get(Servo.class, "gate");

        distance = hardwareMap.get(Rev2mDistanceSensor.class, "distance");

        indices.put("launchSequence", 0);
        indices.put("intakeSequence", 0);
        indices.put("paths", 0);
        indices.put("pathSequence", 0);
        indices.put("launchIntake", 0);
        indices.put("aprilPaths", 0);

        touchSensor = hardwareMap.get(DigitalChannel.class, "touchSensor");
        touchSensor.setMode(DigitalChannel.Mode.INPUT);

        initAprilTag();
        visionPortal.resumeStreaming();
    }

    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

    }


    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                if (detection.id == 21) LastAprilTag = 1;
                if (detection.id == 22) LastAprilTag = 2;
                if (detection.id == 23) LastAprilTag = 3;

                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));

            }
            else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }

        }
        telemetry.addData("AprilTag seen", LastAprilTag);
        telemetry.update();
    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        pathState = 0;
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}
}