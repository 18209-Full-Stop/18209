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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


import java.util.function.Supplier;

@Configurable
@TeleOp
public class BLUERealNoPedro extends OpMode {
    public static double ALLIANCE_HEADING_OFFSET = 0;
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
    private boolean useAuto = true;

    @Override
    public void init() {
        cos = Math.cos(rotate);
        sin = Math.sin(rotate);

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        chooChoo = hardwareMap.get(DcMotor.class, "chooChoo");
        chooChoo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backRight = hardwareMap.get(DcMotor.class, "backRight");
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


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
//        follower.startTeleopDrive(true);
    }

    @Override
    public void loop() {

        x = -gamepad1.left_stick_x;
        y = -gamepad1.left_stick_y;

        rotateX = x * cos - y * sin;
        rotateY = x * sin + y * cos;


        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        frontLeft.setPower(drive + strafe + rotate);
        frontRight.setPower(drive - strafe - rotate);
        backLeft.setPower(drive - strafe + rotate);
        backRight.setPower(drive + strafe - rotate);

//        //Forward and back
//        frontLeft.setPower(-gamepad1.left_stick_y);
//        backLeft.setPower(-gamepad1.left_stick_y);
//        frontRight.setPower(-gamepad1.left_stick_y);
//        backRight.setPower(-gamepad1.left_stick_y);
//
//        //Strafe!
//        frontLeft.setPower(gamepad1.left_stick_x);
//        backLeft.setPower(-gamepad1.left_stick_x);
//        frontRight.setPower(-gamepad1.left_stick_x);
//        backRight.setPower(gamepad1.left_stick_x);
//
//        //Turning
//        frontLeft.setPower(-gamepad1.right_stick_x);
//        backLeft.setPower(-gamepad1.right_stick_x);
//        frontRight.setPower(gamepad1.right_stick_x);
//        backRight.setPower(gamepad1.right_stick_x);


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
        } else if (useAuto || gamepad2.x){
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
        } else{
            chooChoo.setPower(0);
        }

        if(gamepad2.left_trigger>0.3){
            useAuto = false;
        } else if(gamepad2.right_trigger>0.3){
            useAuto = true;
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


        telemetry.addData("distance", distanceINCH);

    }
}