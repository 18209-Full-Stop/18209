package org.firstinspires.ftc.teamcode.pedroPathing.WorkingCode.BLUE;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;

import com.qualcomm.robotcore.hardware.DigitalChannel;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous
public class lowerArm extends OpMode {
    ArrayList<Integer> stages = new ArrayList<>();

    private Follower follower;
    private int pathState;
    private int LastAprilTag;
//    private AprilTagProcessor aprilTag;
//    private VisionPortal visionPortal;

    private DcMotor intake;
    private DcMotor chooChoo;
    private ElapsedTime  runtime = new ElapsedTime();
    private DigitalChannel touchSensor;
    private Timer intakeTimer2;
    private Servo gate;
    private Rev2mDistanceSensor distance;
    double distanceINCH;


    public void LaunchSequence(int state) { //sequence to launch artifacts
        if (stages.get(0) == 0) { //first sets the power to 1 to launch artifact
            chooChoo.setPower(1);
            stages.set(0,1);
        } else if (stages.get(0) == 1) {
            if (distanceINCH <= 3) {
                chooChoo.setPower(0);
                stages.set(0, 0);
                pathState = state;
            }
        }
    }



    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: //forward to launching position
                LaunchSequence(1);
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
        distanceINCH = distance.getDistance(DistanceUnit.INCH);
        telemetry.update();
    }

    @Override
    public void init() {
        gate = hardwareMap.get(Servo.class, "gate");

        distance = hardwareMap.get(Rev2mDistanceSensor.class, "distance");

        LastAprilTag = 1;
        intakeTimer2 = new Timer();
        follower = Constants.createFollower(hardwareMap);

        chooChoo = hardwareMap.get(DcMotor.class, "chooChoo");
        chooChoo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        chooChoo.setPower(0);

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setPower(0);


        stages.add(0); //index for LaunchSequence (0)--runs launcher until hits button
        stages.add(0); //index for IntakeSequence (1)--runs intake for 1 second
        stages.add(0); //index for Paths (2)--follows set path
        stages.add(0);
        stages.add(0); //index for PathSequence (4)--follows 3 consecutive paths
        stages.add(0); //index AprilPaths (5)--same as Paths but updates the index for PathSequence
        stages.add(0); //index for LaunchIntake (6)--runs x3 launch cycle and x2 intake

        touchSensor = hardwareMap.get(DigitalChannel.class, "touchSensor");
        touchSensor.setMode(DigitalChannel.Mode.INPUT);
        initAprilTag();
//        visionPortal.resumeStreaming();
    }

    private void initAprilTag() {

//        // Create the AprilTag processor.
//        aprilTag = new AprilTagProcessor.Builder()
//                .build();
//
//        // Create the vision portal by using a builder.
//        VisionPortal.Builder builder = new VisionPortal.Builder();
//
//        // Set the camera (webcam vs. built-in RC phone camera).
//        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
//
//        // Set and enable the processor.
//        builder.addProcessor(aprilTag);
//
//        // Build the Vision Portal, using the above settings.
//        visionPortal = builder.build();

    }


    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {
//        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
//        telemetry.addData("# AprilTags Detected", currentDetections.size());
//
//        // Step through the list of detections and display info for each one.
//        for (AprilTagDetection detection : currentDetections) {
//            if (detection.metadata != null) {
//                if (detection.id == 21) LastAprilTag = 1;
//                if (detection.id == 22) LastAprilTag = 2;
//                if (detection.id == 23) LastAprilTag = 3;
//
//                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
//                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
//                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
//                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
//
//            }
//            else {
//                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
//                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
//            }
//
//        }
//        telemetry.addData("AprilTag seen", LastAprilTag);
//        telemetry.update();
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