package org.firstinspires.ftc.teamcode.pedroPathing.WorkingCode.TeleOp;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


import java.util.function.Supplier;

@Configurable
@TeleOp
public class BLUEfieldCentricDistance extends OpMode {
    public static double ALLIANCE_HEADING_OFFSET = 0;
    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;
    private DcMotor chooChoo;
    //private DigitalChannel touchSensor;
    private Servo gate;
    private Servo foot;
    private DcMotor backRight;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor frontLeft;
    private Rev2mDistanceSensor distance;
    double distanceINCH;
    double x;
    double y;
    double rotate = -Math.PI/2;
    double cos;
    double sin;
    double rotateX;
    double rotateY;
    private DcMotor intake;
    private boolean position;
    private boolean lastPosition;

    @Override
    public void init() {
        cos = Math.cos(rotate);
        sin = Math.sin(rotate);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

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
    }

    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive(true);
    }

    @Override
    public void loop() {

        x = -gamepad1.left_stick_x;
        y = -gamepad1.left_stick_y;

        rotateX = x * cos - y * sin;
        rotateY = x * sin + y * cos;


        //Call this once per loop
        follower.update();
//        telemetryM.update();

        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors

            //This is the normal version to use in the TeleOp

            if (!slowMode) follower.setTeleOpDrive(
                    rotateY,
                    rotateX,
                    -gamepad1.right_stick_x,
                    false // field centric
            );

                //This is how it looks with slowMode on
            else follower.setTeleOpDrive(
                    rotateY * slowModeMultiplier,
                    rotateX * slowModeMultiplier,
                    -gamepad1.right_stick_x * slowModeMultiplier,
                    false // field centric
            );
        }

        //Automated PathFollowing
//        if (gamepad1.aWasPressed()) {
//            follower.followPath(pathChain.get());
//            automatedDrive = true;
//        }

//        //Stop automated following if the follower is done
//        if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
//            follower.startTeleopDrive();
//            automatedDrive = false;
//        }

//        double dist = distance.getDistance(DistanceUnit.INCH);
//        if (Double.isNaN(dist) || Double.isInfinite(dist)) {
//        } else {
//            distanceINCH = dist;
//        }

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

        if(gamepad1.right_trigger > 0.3){
            foot.setPosition(0);
        } else if (gamepad1.left_trigger > 0.3){
            foot.setPosition(0.5);
        }


//        if(gamepad1.right_bumper && !lastBump){
//            footDown = !footDown;
//            if (footDown){
//                foot.setPosition(0.2);
//            } else{
//                foot.setPosition(0.6);
//            }
//        }
        //lastBump = gamepad1.right_bumper;



        if (gamepad1.left_bumper){
            follower.setPose(new Pose(follower.getPose().getX(), follower.getPose().getY(), 0));
        }

        telemetry.addData("distance", distanceINCH);

    }
}