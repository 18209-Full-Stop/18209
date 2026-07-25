package org.firstinspires.ftc.teamcode.pedroPathing;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


//@Autonomous
public class BLUEAprilTags extends OpMode {
    ArrayList<Integer> stages = new ArrayList<>();

    private Follower follower;
    private int pathState;
    private int LastAprilTag;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private DcMotor intake;
    private DcMotor chooChoo;
    private ElapsedTime  runtime = new ElapsedTime();
    private DigitalChannel touchSensor;

    private Timer intakeTimer2;
    private final Pose startPose = new Pose(56.000, 8.000, Math.toRadians(90)); // Start Pose of our robot.

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


    public void buildPaths() { //sets up all the paths that the robot will follow
        Path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(56.000, 8.000), new Pose(40.429, 102.416))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(45))
                .build();

        Path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(40.429, 102.416), new Pose(42.311, 84.846))
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(180))
                .build();

        Path3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(42.311, 84.846), new Pose(23.883, 84.790))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();


        Path4 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(23.883, 84.790), new Pose(40.429, 102.416))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(45))
                .build();

        Path5 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(40.429, 102.416), new Pose(42.311, 58.951))
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(180))
                .build();

        Path6 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(42.311, 58.951), new Pose(23.544, 59.133))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        Path7 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(23.544, 59.133), new Pose(40.429, 102.416))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(45))
                .build();

        Path8 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(40.429, 102.416), new Pose(42.525, 38.144))
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(180))
                .build();

        Path9 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(42.525, 38.144), new Pose(16.791, 38.144))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        Path10 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(16.791, 38.144), new Pose(40.429, 102.416))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(45))
                .build();

    }

    public void LaunchSequence(int state){
        if (stages.get(0) == 0) { //first sets the power to 1 to launch artifact
            runtime.reset();
            chooChoo.setPower(1.0);
            telemetry.addData("chooChoo", 1);
            stages.set(0,1);
        }
        else if (stages.get(0) == 1) { //stops chooChoo after 1 second
            if (runtime.seconds() >= 1.0) {
                chooChoo.setPower(0);
                telemetry.addData("chooChoo", 0);
                stages.set(0,2);
            }
        } else if (stages.get(0) == 2){ //runs chooChoo until it touches the button, then proceeds to next step
            chooChoo.setPower(1);
            if (!touchSensor.getState()) {
                chooChoo.setPower(0);
                pathState = state;
                stages.set(0, 0);
            }

        }
    }

    public void IntakeSequence(int state){
        if (stages.get(1) == 0){ //runs intake
            telemetry.addData("intake", 1);
            intakeTimer2.resetTimer();
            intake.setPower(1.0);
            stages.set(1,1);
        } else if (stages.get(1) == 1){
            if (intakeTimer2.getElapsedTimeSeconds() >= 1) { //stops intake after 1 second, proceeds to next step
                telemetry.addData("intake", 0);
                intake.setPower(0);
                stages.set(1,0);
                pathState = state;
            }
        }
    }

    public void Paths (PathChain path, int state){
        if (stages.get(2) == 0){ //follows specified path
            follower.followPath(path);
            stages.set(2,1);
        } else if (stages.get(2) == 1){ //next step once path is done
            if (!follower.isBusy()){
                pathState = state;
                stages.set(2,0);
            }
        }
    }

    public void PathsIntake(PathChain path, int state){
        intake.setPower(1.0);
        if (stages.get(3) == 0){ //runs path alongside intake
            follower.followPath(path,.5,true);
            stages.set(3,1);
        } else if (stages.get(3) == 1){ //stops path after path is done, turns off intake, next step
            if (!follower.isBusy()) {
                intake.setPower(0);
                pathState = state;
                stages.set(3,0);
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

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: //forward to launching position
                Paths(Path1, 1);
                break;
            case 1: //first launch
                LaunchSequence(2);
                break;
            case 2: //intake to reload
                IntakeSequence(3);
                break;
            case 3: //second launch
                LaunchSequence(4);
                break;
            case 4: //reload
                IntakeSequence(5);
                break;
            case 5: //third load
                LaunchSequence(6);
                break;
            case 6: //line up to next position based upon what the motif is
                if (LastAprilTag == 1){
                    PathSequence(Path2, Path3, Path4, 7);
                } else if (LastAprilTag == 2){
                    PathSequence(Path5, Path6, Path7, 7);
                } else if (LastAprilTag == 3){
                    PathSequence(Path8, Path9, Path10, 7);
                }
                break;
            case 7: //first launch
                LaunchSequence(8);
                break;
            case 8: //first intake
                IntakeSequence(9);
                break;
            case 9: //second launch
                LaunchSequence(10);
                break;
            case 10: //second intake
                IntakeSequence(11);
                break;
            case 11: //third launch
                LaunchSequence(12);
                break;
            default:
                break;
        }
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        telemetry.addData("touch sensor", touchSensor.getState());
        telemetry.addData("timer intake", intakeTimer2.getElapsedTimeSeconds());

        telemetry.update();
    }

    @Override
    public void init() {
        LastAprilTag = 1;
        intakeTimer2 = new Timer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        chooChoo = hardwareMap.get(DcMotor.class, "chooChoo");
        chooChoo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        chooChoo.setPower(0);

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setPower(0);

        stages.add(0);
        stages.add(0);
        stages.add(0);
        stages.add(0);
        stages.add(0);
        stages.add(0);

        touchSensor = hardwareMap.get(DigitalChannel.class, "touchSensor");
        touchSensor.setMode(DigitalChannel.Mode.INPUT);
        initAprilTag();
        visionPortal.resumeStreaming();
    }

    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }

    private void setAprilTags() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                if (detection.id == 21) LastAprilTag = 1;
                if (detection.id == 22) LastAprilTag = 2;
                if (detection.id == 23) LastAprilTag = 3;
                telemetry.addData("AprilTag seen", LastAprilTag);
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                telemetry.update();
            }
            else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }

        }
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
