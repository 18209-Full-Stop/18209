package org.firstinspires.ftc.teamcode.pedroPathing.WorkingCode.TeleOp;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.HashMap;
import java.util.function.Supplier;

@TeleOp
public class BLUEAutoAprilTag extends OpMode {

    private Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 0, 0, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);

    //variables + lists
    HashMap<String, Integer> indices = new HashMap<String, Integer>();
    private int numBall = 0;
    private static final boolean USE_WEBCAM = true;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private static final double gatePosdown = 0.6;
    private static final double gatePosUp = 0.2;
    private int LastAprilTag = 1;
    double distanceINCH;
    private boolean finished;
    private boolean lower = false;
    private double servoTime = 0.2;
    private double seroTimeOpen = 2.0;

    //declaring motors, servos, + cameras
    private Rev2mDistanceSensor distance;
    private DcMotor intake;
    private DcMotor chooChoo;
    private Servo gate;

    public static double ALLIANCE_HEADING_OFFSET = 0;
    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;
    private Servo foot;
    private DcMotor backRight;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor frontLeft;
    double x;
    double y;
    double rotate = -Math.PI/2;
    double cos;
    double sin;
    double rotateX;
    double rotateY;

    private boolean position;
    private boolean lastPosition;
    private boolean auto = false;

    //positions for follower
    private int pathState;
    //hello
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

    double kP = 0.05; //used to be 0.002, then 0.01
    double error = 0;
    double lastError = 0;
    double goalX = 30; //offset here
    double angleTolerance = 0.3; //used to be 0.2
    double kD = 0.0001; //used to be 0.0001, then 0.001, then 0.005
    double curTime = 0;
    double lastTime = 0;
    double dTerm = 0;
    double dT = 0;

    double lockedGoal = 0;
    boolean goalLocked = false;



    @Override
    public void loop() {

        x = -gamepad1.left_stick_x;
        y = -gamepad1.left_stick_y;

        rotateX = x * cos - y * sin;
        rotateY = x * sin + y * cos;

        if (gamepad1.right_trigger > 0.3) {
            auto = true;
        } else {
            auto = false;
        }

        if (!auto) {
            rotate = -gamepad1.right_stick_x;
        }

        //Call this once per loop
        follower.update();
//        telemetryM.update();

        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors

            //This is the normal version to use in the TeleOp

                follower.setTeleOpDrive(
                        rotateY,
                        rotateX,
                        rotate,
                        false // field centric
                );
        }

        //Slow Mode
        if (gamepad1.rightBumperWasPressed()) {
            slowMode = !slowMode;
        }

        //Optional way to change slow mode strength
        if (gamepad1.xWasPressed()) {
            slowModeMultiplier += 0.25;
        }

        //Optional way to change slow mode strength
        if (gamepad1.yWasPressed()) {
            slowModeMultiplier -= 0.25;
        }

        if (gamepad2.a) {//this will run the choo-choo no matter what in order to launch it
            chooChoo.setPower(1);
        } else if (gamepad2.x) { //gamepad2.x will run chooChoo until it touches the button

            double dist = distance.getDistance(DistanceUnit.INCH);
            if (Double.isNaN(dist) || Double.isInfinite(dist)) {
            } else {
                distanceINCH = dist;
            }

            if ((distanceINCH >= 3 && distanceINCH < 30 && Double.isFinite(distanceINCH))) { //if it is not touching the button, it will run //used to be 3.45
                chooChoo.setPower(1);
            } else { //once it touches the button, it stops (goes to lowest point)
                chooChoo.setPower(0);
            }
        } else { //if none of these buttons are pressed it stops
            chooChoo.setPower(0);
        }

        if (gamepad2.left_bumper) { //left bumper gets artifacts out (brings them down)
            intake.setPower(-1);
        } else if (gamepad2.right_bumper) { //right bumper puts them up
            intake.setPower(1);
        } else { //otherwise turns intake off
            intake.setPower(0);
        }

        if(gamepad2.y){
            gate.setPosition(0.2);
        } else {
            gate.setPosition(0.6);
        }

        if(gamepad1.right_bumper){
            foot.setPosition(0);
        } else if (gamepad1.left_bumper){
            foot.setPosition(0.5);
        }

        if (gamepad1.left_trigger>0.3){
            follower.setPose(new Pose(follower.getPose().getX(), follower.getPose().getY(), 0));
        }

        telemetry.addData("distance", distanceINCH);


        telemetry.addData("Camera State", visionPortal.getCameraState());
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        AprilTagDetection targetTag = null;
        for (AprilTagDetection detection : currentDetections) {
            telemetry.addData("april seen id", detection.id);
            telemetry.addData("aprillll", detection);
            if (detection.id == 20) {
                LastAprilTag = 0;
                targetTag = detection;
            }
        }
        telemetry.addData("Tags detected", currentDetections.size());

        if (gamepad1.right_trigger>0.3){
            if (targetTag != null){
                telemetry.addData("yessir2", 12);
                telemetry.addData("angle", error);
//                if (!goalLocked) {
//                    lockedGoal = targetTag.ftcPose.bearing;
//                    goalLocked = true;
//                }

//                error = lockedGoal - targetTag.ftcPose.bearing;
                error = goalX - targetTag.ftcPose.bearing;
                if (Math.abs(error) < angleTolerance){
                    telemetry.addData("turning",0);
                    rotate = 0;
                } else {
                    telemetry.addData("turning",1);
                    double pTerm = error * kP;

                    curTime = getRuntime();
                    dT = curTime - lastTime;


                    if(dT > 0.01){
                        dTerm = ((error-lastError)/dT) * kD;
                    }


                    rotate = Range.clip(pTerm + dTerm, -0.4, 0.4);

                    lastError = error;
                    lastTime = curTime;

                }
            } else {
                telemetry.addData("yessir2", 0);
                lastTime = getRuntime();
                lastError = 0;
            }
        } else {
            telemetry.addData("yessir2", 1);
            lastError = 0;
            lastTime = getRuntime();
        }
    }

    @Override
    public void init() {
        cos = Math.cos(rotate); //used to be in init loop
        sin = Math.sin(rotate);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();


        //telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();

        chooChoo = hardwareMap.get(DcMotor.class, "chooChoo");
        chooChoo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        gate = hardwareMap.get(Servo.class, "gate");
        foot = hardwareMap.get(Servo.class, "foot");

        distance = hardwareMap.get(Rev2mDistanceSensor.class, "distance");

        position = true;
        lastPosition = false;

        telemetry.addLine("Initialized");

        telemetry.update();

        initAprilTag();
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
                .setCameraPose(cameraPosition, cameraOrientation)

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
        aprilTag.setDecimation(1);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

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

    }   // end method initAprilTag()

    /**
     * Add telemetry about AprilTag detections.
     */
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                // Only use tags that don't have Obelisk in them
                if (!detection.metadata.name.contains("Obelisk")) {
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                            detection.robotPose.getPosition().x,
                            detection.robotPose.getPosition().y,
                            detection.robotPose.getPosition().z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
                            detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
                            detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
                            detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
                }
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");

    }   // end method telemetryAprilTag()




/** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        resetRuntime();
        curTime = getRuntime();
        follower.startTeleopDrive(true);

        if(visionPortal != null){
            visionPortal.resumeStreaming();
        }
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}
}