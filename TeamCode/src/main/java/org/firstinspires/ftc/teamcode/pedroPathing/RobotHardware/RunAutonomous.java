package org.firstinspires.ftc.teamcode.pedroPathing.RobotHardware;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Timer;

public class RunAutonomous {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    private final Pose startPoseB = new Pose(0 ,0 ,Math.toRadians(0));
    private final Pose startPoseR = new Pose(0 ,0 ,Math.toRadians(0));

    private final Pose scorePoseR = new Pose(75, 70, Math.toRadians(0));
    private final Pose scorePoseB = new Pose(75, 70, Math.toRadians(0));


    private PathChain move1;

    public RunAutonomous(boolean red, HardwareMap hwmap){

        follower = Constants.createFollower(hwmap);
        pathTimer = new Timer();

        if (red){
            buildPathsRed();
            follower.setStartingPose(startPoseR);
        }else {
            buildPathsBlue();
            follower.setStartingPose(startPoseB);
        }
    }


    private void buildPathsBlue(){
        move1 = follower.pathBuilder()
                .addPath(new BezierLine(startPoseR, scorePoseR))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }

    private void buildPathsRed(){
        move1 = follower.pathBuilder()
                .addPath(new BezierLine(startPoseB, scorePoseB))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }

    public void autonomousPathUpdateBlue(){
        switch (pathState){
            case 1:
                follower.followPath(move1);
                break;
            case 2:
                break;
        }
    }

    public void autonomousPathUpdateRed(){
        switch (pathState){
            case 1:
                follower.followPath(move1);
                break;
            case 2:
                break;
        }

    }

    public void startMode(){
        pathState = 1;
    }
}
