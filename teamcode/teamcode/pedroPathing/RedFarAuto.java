package org.firstinspires.ftc.teamcode.pedroPathing;

import static android.os.SystemClock.sleep;

import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.main.HardwareManagerNew;


@Autonomous(name = "Red FarPedro Pathing Autonomous", group = "Autonomous")
public class RedFarAuto extends OpMode {

    private HardwareManagerNew hardwareManager = null;

    public Follower follower; // Pedro Pathing follower instance
    private Timer pathTimer, opModeTimer; // Timer for the autonomous path
    private int pathState; // Current autonomous path state (state machine)
    // poses
    private final Pose startPose = new Pose(88, 10, Math.toRadians(90));
    private final Pose turnStart = new Pose(89, 10, Math.toRadians(69));
    private final Pose readyPickup1 = new Pose(104, 36, Math.toRadians(180));
    private final Pose pickup1 = new Pose(135, 36, Math.toRadians(180));
    private final Pose shootReturn1 = new Pose(88, 10, Math.toRadians(69));
    private final Pose readyPickup2 = new Pose(135, 24, Math.toRadians(90));
    private final Pose pickup2 = new Pose(135, 24, Math.toRadians(90));
    private final Pose shootReturn2 = new Pose(88, 10, Math.toRadians(69));
    private final Pose leaveEnd = new Pose(110, 10, Math.toRadians(90));

    // path chain
    private Path Path1;
    private PathChain Path2, Path3, Path4, Path5, Path6, Path7, Path8;
    private boolean flyWheelOn;

    public void buildPaths(){
        Path1 = new Path(new BezierLine(startPose, turnStart));
        Path1.setLinearHeadingInterpolation(startPose.getHeading(), turnStart.getHeading());
        Path2 = follower.pathBuilder()
                .addPath(new BezierLine(turnStart, readyPickup1))
                .setLinearHeadingInterpolation(turnStart.getHeading(), readyPickup1.getHeading())
                .build();
        Path3 = follower.pathBuilder()
                .addPath(new BezierLine(readyPickup1, pickup1))
                .setLinearHeadingInterpolation(readyPickup1.getHeading(), pickup1.getHeading())
                .build();
        Path4 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1, shootReturn1))
                .setLinearHeadingInterpolation(pickup1.getHeading(), shootReturn1.getHeading())
                .build();
        Path5 = follower.pathBuilder()
                .addPath(new BezierLine(shootReturn1, readyPickup2))
                .setLinearHeadingInterpolation(shootReturn1.getHeading(), readyPickup2.getHeading())
                .build();
        Path6 = follower.pathBuilder()
                .addPath(new BezierLine(readyPickup2, pickup2))
                .setLinearHeadingInterpolation(readyPickup2.getHeading(), pickup2.getHeading())
                .build();
        Path7 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2, shootReturn2))
                .setLinearHeadingInterpolation(pickup2.getHeading(), shootReturn2.getHeading())
                .build();
        Path8 = follower.pathBuilder()
                .addPath(new BezierLine(shootReturn2, leaveEnd))
                .setLinearHeadingInterpolation(shootReturn2.getHeading(), leaveEnd.getHeading())
                .build();
    }
    public void statePathUpd() {
        switch (pathState){
            case 0:
                this.autoAllShoot();
                follower.followPath(Path1);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()){
                    this.autoAllShoot();
                    follower.followPath(Path2, true);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()){
                    follower.followPath(Path3, true);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()){
                    this.autoAllShoot();
                    follower.followPath(Path4, true);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()){
                    this.autoAllShoot();
                    follower.followPath(Path5, true);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()){
                    follower.followPath(Path6, true);
                    setPathState(6);
                }
                break;
            case 6:
                if(!follower.isBusy()){
                    this.autoAllShoot();
                    follower.followPath(Path7, true);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()){
                    this.autoAllShoot();
                    follower.followPath(Path8, true);
                    setPathState(8);
                }
                break;
            default:
                telemetry.addLine("No State");
                break;
        }
    }
    public void setPathState(int newState){
        pathState = newState;
        pathTimer.resetTimer();
    }
    @Override
    public void init() {
        pathTimer = new Timer();
        opModeTimer = new Timer();
        opModeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
        telemetry.addData("Status", "Initialized");
    }
    @Override
    public void start(){
        opModeTimer.resetTimer();
        setPathState(0);
    }
    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        // Log values to Panels and Driver Station
        statePathUpd();
        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.update();
        hardwareManager = new HardwareManagerNew(this);
        this.hardwareManager.fly(this.flyWheelOn, false);
    }

    private void autoShootImpl(){
        this.hardwareManager.stopDrive();
        this.hardwareManager.blockPos(true);
        this.hardwareManager.intake(true);
        sleep((long) HardwareManagerNew.SHOOT_TIME);
        this.hardwareManager.intake(false);
        this.hardwareManager.blockPos(false);
        this.telemetry.update();
    }
    private void autoAllShoot(){
        for (int i = 0; i < HardwareManagerNew.MAX_BALL_CAPACITY; i++){
            this.autoShootImpl();
        }
        this.hardwareManager.fly(false);
    }
}