package org.firstinspires.ftc.teamcode.pedroPathing.WorkingCode.RED.GoalStart;


import com.pedropathing.geometry.BezierCurve;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.HashMap;

@Autonomous
public class REDStartTriangleNoWaitReducedTimeApril extends OpMode {
    //variables + lists
    HashMap<String, Integer> indices = new HashMap<String, Integer>();
    private int numBall = 0;
    private static final double gatePosdown = 0.6;
    private static final double gatePosUp = 0.2;
    private int LastAprilTag = 1;
    double distanceINCH;
    private boolean finished;
    private boolean lower = false;
    private double servoTime = 0.2;
    private double seroTimeOpen = 2.0;
    private double timeOut = 4.0;

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
    private Timer aprilTimer;
    private Timer aprilIntakeTimer;
    private double intakeTimer;

    //positions for follower
    private Follower follower;
    private int pathState;
    private final Pose startPose = new Pose(56.10261370205579, 6.691190706679565, Math.toRadians(90)); // Start Pose of our robot. used to be 25.762, 128.207
    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path4;
    public PathChain Path5;

    public PathChain Path6;
    public PathChain Path7;
    public PathChain Path8;

    public PathChain Path9;
    public PathChain Path10;
    public PathChain Path11;
    public PathChain PathPark;


    public void buildPaths() { //sets up all the paths that the robot will follow
        //moves forward to launching position
        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose((72-56.10261370205579)+72, 6.691190706679565),

                                new Pose((72-51.036371392722714)+72, 95.0865882696608)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(137))

                .build();

        //moves to pick up more + rotate 180
        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose((72-51.036371392722714)+72, 95.0865882696608),

                                new Pose((72-49.094)+72, 86.459)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(137), Math.toRadians(0))

                .build();

        //picks up artifacts
        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                //new Pose(49.094, 86.459),
                                new Pose((72-49.094)+72, 86.459),

                                new Pose((72-14)+72, 86.459) //used to be 19.035164328657316
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        //returns to launching position
        Path4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose((72-14)+72, 86.459),

                                new Pose((72-51.036371392722714)+72, 95.0865882696608)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(137))

                .build();


        Path5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(follower.getPose().getX(), follower.getPose().getY()),

                                new Pose((72-51.036371392722714)+72, 95.0865882696608)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(137))

                .build();
//this wasn't here before

        //parks to face 90 degrees
        PathPark = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose((72-1.036371392722714)+72, 95.0865882696608),

                                new Pose((72-45.59483870967742)+72, 73.16129032258064)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(137), Math.toRadians(90))

                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose((72-51.036371392722714)+72, 95.0865882696608),

                                new Pose((72-51.036371392722714), 62.918)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(137), Math.toRadians(0))

                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose((72-51.036)+72, 60.241),

                                new Pose((72-7.324)+72, 59.910)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        Path8 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose((72-7.324)+72, 59.910),
                                new Pose((72-68.654)+72, 42.937),
                                new Pose((72-51.036)+72, 95.087)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(137))

                .build();

        Path9 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose((72-51.036371392722714)+72, 95.0865882696608),

                                new Pose((72-50.404)+72, 35.594)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(137), Math.toRadians(0))

                .build();

        Path10 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose((72-50.404)+72, 35.594),

                                new Pose((72-7.265)+72, 38.71519554695063)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        Path11 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose((72-7.265)+72, 38.71519554695063),

                                new Pose((72-51.036371392722714)+72, 95.0865882696608)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(137))

                .build();
    }

    public void launchIntakeCycle (int state, double time){
        if (indices.get("launchIntakeCycle") == 0){
            launcher(1); //1st lower
        } else if (indices.get("launchIntakeCycle") == 1){
            lowerLauncher(2); //1st launch
        } else if (indices.get("launchIntakeCycle") == 2){
            intake(3, time); //intake to raise one more
        } else if (indices.get("launchIntakeCycle") == 3){
            indices.put("launchIntakeCycle", 0);
            indices.put("LaunchIntake", state); //reset, next step
        }
    }

    public void launcher(int state){
        if(indices.get("launch") == 0){
            runtime.reset();
            chooChoo.setPower(1);
            indices.put("launch",1);
        } else if(indices.get("launch") == 1) {
            if (runtime.seconds() >= 0.6) { //used to be 0.75
                chooChoo.setPower(0);
                indices.put("launch", 2);
            }
        } else if (indices.get("launch") == 2){
            indices.put("launch", 0);
            indices.put("launchIntakeCycle", state);
        }
    }

    public void intake(int state, double timer){
        if (indices.get("intake") == 0){
            intakeTimer2.resetTimer();
            intake.setPower(1);
            gate.setPosition(gatePosUp);
            indices.put("intake", 1);
        } else if (indices.get("intake") == 1 && intakeTimer2.getElapsedTimeSeconds() >= timer){
            gate.setPosition(gatePosdown);
            intake.setPower(-1);
            intakeTimer2.resetTimer();
            indices.put("intake", 2);
        } else if (indices.get("intake") == 2 && intakeTimer2.getElapsedTimeSeconds() >= 0.2){
            intake.setPower(0);
            intakeTimer2.resetTimer();
            indices.put("intake", 3);
        } else if (indices.get("intake") == 3 && intakeTimer2.getElapsedTimeSeconds() >= 0.2){
            indices.put("intake", 0);
            indices.put("launchIntakeCycle", state);
        }
    }
    public void lowerLauncher(int state) {
        if (indices.get("lowerLauncher") == 0){
            launchTimer2.resetTimer();
            indices.put("lowerLauncher", 1);
        } else if (indices.get("lowerLauncher") == 1){
            double dist = distance.getDistance(DistanceUnit.INCH);
            if (Double.isNaN(dist) || Double.isInfinite(dist)) {
            } else {
                distanceINCH = dist;
            }
            if ((distanceINCH <= 3 && distanceINCH > 0 && Double.isFinite(distanceINCH)) || launchTimer2.getElapsedTimeSeconds() >= 1) { //used to be 2
                chooChoo.setPower(0);
                telemetry.addData("chooChoo", 4);
                indices.put("lowerLauncher", 0);
                indices.put("launchIntakeCycle", state);
            }
        }
    }

    public void lowerLauncherForIntake(int state){
        if (indices.get("lowerLauncherForIntake") == 0){
            launchTimer2.resetTimer();
            indices.put("lowerLauncherForIntake", 1);
        } else if (indices.get("lowerLauncherForIntake") == 2){
            double dist = distance.getDistance(DistanceUnit.INCH);
            if (Double.isNaN(dist) || Double.isInfinite(dist)) {
            } else {
                distanceINCH = dist;
            }
            if ((distanceINCH <= 3 && distanceINCH > 0 && Double.isFinite(distanceINCH)) || launchTimer2.getElapsedTimeSeconds() >= 1) { //used to be 2
                chooChoo.setPower(0);
                telemetry.addData("chooChoo", 4);
                indices.put("lowerLauncherForIntake", 0);
                indices.put("launchIntake", state);
            }
        }
    }

//    public void LaunchSequence(int state){
//        if (indices.get("launchSequence") == 0) { //resets timer
//            telemetry.addData("chooChoo", 0);
//            runtime.reset();
//            indices.put("launchSequence",1);
//        } else if (indices.get("launchSequence") == 1 && runtime.seconds() > .3) { //waits 0.5 seconds, then runs chooChoo, used to be 0.5
//            runtime.reset();
//            chooChoo.setPower(1.0);
//            telemetry.addData("chooChoo", 1);
//            indices.put("launchSequence",2);
//        } else if (indices.get("launchSequence") == 2 && runtime.seconds() >= 0.65) { //stops chooChoo after 1 second, used to be 1, then 0.75
//            chooChoo.setPower(0);
//            telemetry.addData("chooChoo", 2);
//            launchTimer2.resetTimer();
//            indices.put("launchSequence",3);
//        } else if (indices.get("launchSequence") == 3){ //runs chooChoo until it is lowered all the way, then proceeds to next step
//            telemetry.addData("chooChoo", 3);
//            chooChoo.setPower(1);
//            double dist = distance.getDistance(DistanceUnit.INCH);
//            if (Double.isNaN(dist) || Double.isInfinite(dist)) {
//            } else {
//                distanceINCH = dist;
//            }
//            if ((distanceINCH <= 3 && distanceINCH > 0 && Double.isFinite(distanceINCH)) || launchTimer2.getElapsedTimeSeconds() >= 1 ) { //used to be 2
//                chooChoo.setPower(0);
//                telemetry.addData("chooChoo", 4);
//                indices.put("launchSequence",0);
//                indices.put("launchIntake",state);
//            }
//        }
//    }

    //intake sequence
//    public void IntakeSequence(int state, double timer, double timer2) {
//        if (indices.get("intakeSequence") == 0) { //runs intake
//            telemetry.addData("intake", 0);
//            intakeTimer2.resetTimer();
//            intake.setPower(1.0);
//            numBall++;
//            gate.setPosition(gatePosUp);
//            indices.put("intakeSequence", 1);
//        } else if (indices.get("intakeSequence") == 1) {
//            if (numBall == 2) { //if it is on its last ball, it will wait 0.8 seconds + run intake
//                if (intakeTimer2.getElapsedTimeSeconds() >= 0.8) { //used to be 0.8
//                    gate.setPosition(0); //opens the gate more to let out artifact
//                    numBall = 3;
//                }
//            } else if (numBall == 3) { //then it will wait 1.3 seconds and close the gate
//                if (intakeTimer2.getElapsedTimeSeconds() >= 1.0) { //used to be 1.3
//                    telemetry.addData("intake", 1);
//                    indices.put("intakeSequence", 2);
//                }
//            } else if (numBall < 2) { //if it is on its first ball it will raise it for 0.8 seconds
//                if (intakeTimer2.getElapsedTimeSeconds() >= timer) { //used to be 0.8
//                    telemetry.addData("intake", 1);
//                    indices.put("intakeSequence", 2);
//                }
//            }
//        } else if (indices.get("intakeSequence") == 2) {
//            telemetry.addData("intake", 2);
//            intake.setPower(-1);
//            gate.setPosition(gatePosdown);
//            intakeTimer2.resetTimer();
//            indices.put("intakeSequence", 4);
//        } else if (indices.get("intakeSequence") == 3) {
//            indices.put("intakeSequence", 4);
//        } else if (indices.get("intakeSequence") == 4) {
//            if (intakeTimer2.getElapsedTimeSeconds() >= timer2) { //used to be 0.8
//                intake.setPower(0);
//                telemetry.addData("intake", 1);
//                indices.put("intakeSequence", 0);
//                indices.put("launchIntake", state);
//            }
//        }
//    }


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
            if (pathTimer.getElapsedTimeSeconds() >= 0.1){ //waits 1 second, used to be 1
                pathState = state;
                indices.put("paths",0);
            }
        }
    }

    public void AprilPaths(PathChain path, int state, boolean intaking) {
        if (indices.get("aprilPaths") == 0) {//runs path first
            aprilIntakeTimer.resetTimer();
            aprilTimer.resetTimer();
            indices.put("aprilPaths", 1);
        } else if (indices.get("aprilPaths") == 1) {
            if (intaking) {
                follower.followPath(path, .5, true); //used to be 0.35
                intake.setPower(1.0);
                aprilTimer.resetTimer();
            } else {
                follower.followPath(path);
            }
            indices.put("aprilPaths", 2);
        } else if (indices.get("aprilPaths") == 2) {
            if (intaking) {
                aprilIntakeTimer.resetTimer();
                aprilTimer.resetTimer();
                indices.put("aprilPaths", 3);

            } else indices.put("aprilPaths", 3);
        }
        else if (indices.get("aprilPaths") == 3) { //stops path after path is done, next step
            if (intaking){
                if (follower.isBusy() && aprilTimer.getElapsedTimeSeconds() >= 2.5){ //used to be 4
                    finished = false;
                    follower.breakFollowing();
                    aprilIntakeTimer.resetTimer();
                    indices.put("aprilPaths",4);
                    gate.setPosition(gatePosUp);
                } else if (!follower.isBusy() && aprilTimer.getElapsedTimeSeconds() >= 2){
                    gate.setPosition(gatePosUp);
                    finished = true;
                    aprilIntakeTimer.resetTimer();
                    indices.put("aprilPaths", 4);
                }
            } else {
                if (!follower.isBusy() && aprilTimer.getElapsedTimeSeconds() >= 0.4) { //used to be 1
                    finished = true;
                    aprilIntakeTimer.resetTimer();
                    indices.put("aprilPaths",0);
                    indices.put("pathSequence", state);
                }
            }
        } else if (indices.get("aprilPaths") == 4) {
            gate.setPosition(gatePosdown);
            aprilTimer.resetTimer();
            intake.setPower(0);
            indices.put("aprilPaths", 0);
            indices.put("pathSequence", state);
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
            if (finished){
                AprilPaths(path3, 3, false); //used to not have finished
            } else {
                AprilPaths(Path5, 3, false);
            }
        } else if (indices.get("pathSequence") == 3){
            telemetry.addData("path", 4);
            indices.put("pathSequence", 0);
            pathState = state;
        }
    }

    public void LaunchIntake(int state, double time1, double time2){
        if (indices.get("launchIntake") == 0) { //first launch
            launchIntakeCycle(1, 0.5);
        } else if (indices.get("launchIntake") == 1) { //first intake
            launchIntakeCycle(2, 0.5); //used to be 0.8
        } else if (indices.get("launchIntake") == 2){ //second launch
            lowerLauncherForIntake(3);
        } else if (indices.get("launchIntake") == 3){
            indices.put("launchIntake",0);
            pathState = state;
        }
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: //forward to launching position
                Paths(Path1,1);
                break;
            case 1: //cycle of launching 3 artifacts
                LaunchIntake(2, 0.5, 0.3);
                break;
            case 2: //follows path to pick up more artifacts
                PathSequence(Path9, Path10, Path11, 3);
                break;
            case 3: //2nd cycle of launching 3 artifacts
                LaunchIntake(4, 0.8, 0.3);
                break;
            case 4: //pick up 3 more artifacts
                PathSequence(Path2, Path3, Path4, 5);
                break;
            case 5:
                LaunchIntake(6, 0.5, 0.3);
                break;
            case 6:
                Paths(PathPark, 7);
                break;
            default:
                break;
        }
    }

    @Override
    public void loop() {
//        distanceINCH = distance.getDistance(DistanceUnit.INCH); //used to be line 320
        follower.update();
        autonomousPathUpdate();
        telemetry.addData("timer intake", intakeTimer2.getElapsedTimeSeconds());
        telemetry.addData("timer launch", launchTimer2.getElapsedTimeSeconds());
        telemetry.addData("gateposition", gate.getPosition());
        telemetry.addData("distance", distanceINCH);
        telemetry.addData("april seen", LastAprilTag);



        telemetry.update();
    }

    @Override
    public void init() {
//        LastAprilTag = 1;
        intakeTimer2 = new Timer();
        launchTimer2 = new Timer();
        pathTimer = new Timer();
        aprilTimer = new Timer();
        aprilIntakeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths(); //used to be before startPose
        telemetry.addData("starting", follower.getPose());

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
        indices.put("launchIntakeCycle", 0);
        indices.put("launch", 0);
        indices.put("intake", 0);
        indices.put("lowerLauncher", 0);
        indices.put("lowerLauncherForIntake", 0);

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
        init_loop(); //used to not be here

    }


    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());


        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            telemetry.addData("april seen id", detection.id);
            if (detection.id == 21) LastAprilTag = 1;
            if (detection.id == 22) LastAprilTag = 2;
            if (detection.id == 23) LastAprilTag = 3;
            if (detection.metadata != null) {
                telemetry.addData("april seen id", detection.id);
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