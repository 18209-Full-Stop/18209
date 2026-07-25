package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

//@Autonomous
public class SK_Auto extends OpMode {


    private Follower follower;
    private int pathState;

    private final Pose startPose = new Pose(86.89230769230768, 9.353846153846153, Math.toRadians(90)); // Start Pose of our robot.

    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path3_5;
    public PathChain Path3_5_5;
    public PathChain Path4;

    public void buildPaths() {


        Path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(86.646, 9.354), new Pose(72.123, 71.877))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(45))
                .build();

        Path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(72.123, 71.877),
                                new Pose(83.938, 82.708),
                                new Pose(104.123, 83.692)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .build();

        Path3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(104.123, 83.692),
                                new Pose(108, 83.692)
                        )
                )
                .setVelocityConstraint(5)

                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();
        Path3_5 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(104.123, 83.692),
                                new Pose(114, 83.692)
                        )
                )
                .setVelocityConstraint(5)

                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        Path3_5_5 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(104.123, 83.692),
                                new Pose(122.338, 83.692)
                        )
                )
                .setVelocityConstraint(5)

                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Path4 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(122.338, 83.692),
                                new Pose(89.600, 83.446),
                                new Pose(72.123, 72.123)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))

                .build();
    }


    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(Path1);
                pathState = 1;
                break;
            case 1:
                if (!follower.isBusy()) {
                    pathState = 2;
                }
                break;
            case 2:
                follower.followPath(Path2);
                pathState = 3;
                break;

            case 3:
                if (!follower.isBusy()) {
                    pathState = 3;
                }
                break;
            default:
                break;
        }
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
    }

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

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
