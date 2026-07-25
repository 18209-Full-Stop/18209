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

import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//@Autonomous
public class BLUEGoalDistanceMethods extends OpMode {
    //variables + lists
    ArrayList<Integer> stages = new ArrayList<>();
    private boolean intakingBalls = false;
    private int numBall = 0;
    private static final double gatePosdown = 0.5;
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

    //timers
    private ElapsedTime  runtime = new ElapsedTime();
    private Timer intakeTimer2;
    private Timer pathTimer;
    private Timer launchTimer2;

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

                                new Pose(58.769, 86.375)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(44))

                .build();

        //moves to pick up more + rotate 180
        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(58.769, 86.375),

                                new Pose(49.094, 86.459)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(44), Math.toRadians(180))

                .build();

        //picks up artifacts
        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(49.094, 86.459),

                                new Pose(25.153, 86.459)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        //returns to launching position
        Path4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(25.153, 86.459),

                                new Pose(58.769, 86.375)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(44))

                .build();

        //parks to face 90 degrees
        PathPark = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(58.769, 86.375),

                                new Pose(40, 60)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(44), Math.toRadians(90))

                .build();
    }

    public void LaunchSequence(int state){
        if (stages.get(0) == 0) { //resets timer
            runtime.reset();
            stages.set(0,1);
        }
        if (stages.get(0) == 1 && runtime.seconds() > .5) { //waits 0.5 seconds, then runs chooChoo
            runtime.reset();
            chooChoo.setPower(1.0);
            telemetry.addData("chooChoo", 1);
            stages.set(0,2);
        }
        else if (stages.get(0) == 2) { //stops chooChoo after 1 second
            if (runtime.seconds() >= 1.0) {
                chooChoo.setPower(0);
                telemetry.addData("chooChoo", 0);
                launchTimer2.resetTimer();
                stages.set(0,3);
            }
        } else if (stages.get(0) == 3){ //runs chooChoo until it is lowered all the way, then proceeds to next step
            chooChoo.setPower(1);
            if ((distanceINCH <= 2 && distanceINCH > 0) || launchTimer2.getElapsedTimeSeconds() >= 4 ) {
                chooChoo.setPower(0);
                stages.set(0, 0);
                stages.set(6,state);
            }
        }
    }

    //intake sequence
    public void IntakeSequence(int state) {
        if (stages.get(1) == 0) { //runs intake
            telemetry.addData("intake", 1);
            intakeTimer2.resetTimer();
            intake.setPower(1.0);
            numBall++;
            gate.setPosition(gatePosUp);
            stages.set(1, 1);
        } else if (stages.get(1) == 1) {
            if (numBall == 2) { //if it is on its last ball, it will wait 0.8 seconds + run intake
                if (intakeTimer2.getElapsedTimeSeconds() >= 0.8) {
                    gate.setPosition(0); //opens the gate more to let out artifact
                    numBall = 3;
                }
            } else if (numBall == 3) { //then it will wait 1.3 seconds and close the gate
                if (intakeTimer2.getElapsedTimeSeconds() >= 1.3) {
                    stages.set(1, 2);
                }
            } else { //if it is on its first ball it will raise it for 0.8 seconds
                if (intakeTimer2.getElapsedTimeSeconds() >= 0.8) {
                    stages.set(1, 2);
                }
            }
        } else if (stages.get(1) == 2){
            telemetry.addData("intake", 0);
            intake.setPower(0);
            gate.setPosition(gatePosdown);
            stages.set(1,0);
            stages.set(6, state); //proceed to next step in main program
        }
    }


    public void Paths (PathChain path, int state){
        if (stages.get(2) == 0){ //follows specified path
            if (intakingBalls) follower.followPath(path, .5, true);
            else follower.followPath(path);
            stages.set(2,1);
        } else if (stages.get(2) == 1){ //next step once path is done
            if (!follower.isBusy()){
                stages.set(2,2);
                pathTimer.resetTimer();
            }
        }
        else if (stages.get(2) == 2){ //next step once path is done
            if (pathTimer.getElapsedTimeSeconds() > 1){ //waits 1 second
                pathState = state;
                stages.set(2,0);
            }
        }
    }

    public void AprilPaths(PathChain path, int state) {
        if (stages.get(5) == 0) { //runs path first
            follower.followPath(path);
            stages.set(5, 1);
        } else if (stages.get(5) == 1) { //stops path after path is done, next step
            if (!follower.isBusy()) {
                stages.set(5, 0);
                stages.set(4, state);
            }
        }
    }

    public void PathSequence(PathChain path1, PathChain path2, PathChain path3, int state){
        if (stages.get(4) == 0){
            telemetry.addData("path", 1);
            AprilPaths(path1, 1);
        } else if (stages.get(4) == 1){
            intake.setPower(1);
            telemetry.addData("path", 2);
            AprilPaths(path2, 2);
        } else if (stages.get(4) == 2){
            telemetry.addData("path", 3);
            intake.setPower(0);
            AprilPaths(path3, 3);
        } else if (stages.get(4) == 3){
            telemetry.addData("path", 4);
            stages.set(4, 0);
            pathState = state;
        }
    }

    public void LaunchIntake(int state){
        if (stages.get(6) == 0) { //first launch
            LaunchSequence(1);
        } else if (stages.get(6) == 1) { //first intake
            IntakeSequence(2);
        } else if (stages.get(6) == 2){ //second launch
            LaunchSequence(3);
        } else if (stages.get(6) == 3){ //second intake
            IntakeSequence(4);
        } else if (stages.get(6) == 4){ //third launch
            LaunchSequence(5);
        } else if (stages.get(6) == 5){
            stages.set(6, 0);
            pathState = state;
        }
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: //forward to launching position
                Paths(Path1, 1);
                break;
            case 1: //first launch
                LaunchIntake(2);
                break;
            case 2: //third load
                numBall = 0;
                Paths(Path2, 3);
                break;
            case 3: //third load
                intake.setPower(1);
                intakingBalls = true;
                Paths(Path3, 4);
                break;
            case 4: //third load
                intake.setPower(0);
                intakingBalls = false;
                Paths(Path4, 5);
                break;
            case 5:
                LaunchIntake(6);
                break;
            case 6: //park
                Paths(PathPark, 7);
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
        telemetry.addData("timer intake", launchTimer2.getElapsedTimeSeconds());


        telemetry.update();
    }

    @Override
    public void init() {
        LastAprilTag = 1;
        intakeTimer2 = new Timer();
        launchTimer2 = new Timer();
        pathTimer = new Timer();
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

        stages.add(0); //index for LaunchSequence (0)--runs launcher until hits button
        stages.add(0); //index for IntakeSequence (1)--runs intake for 1 second
        stages.add(0); //index for Paths (2)--follows set path
        stages.add(0);
        stages.add(0); //index for PathSequence (4)--follows 3 consecutive paths
        stages.add(0); //index AprilPaths (5)--same as Paths but updates the index for PathSequence
        stages.add(0); //index for LaunchIntake (6)--runs x3 launch cycle and x2 intake

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