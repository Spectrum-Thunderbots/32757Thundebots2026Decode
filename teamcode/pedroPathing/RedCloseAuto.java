package org.firstinspires.ftc.teamcode.pedroPathing;

import static android.os.SystemClock.sleep;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.main.HardwareManagerNew;

@Autonomous(name = "Red Close Pedro Pathing Autonomous", group = "Autonomous")
public class RedCloseAuto extends OpMode {

    private HardwareManagerNew hardwareManager = null;
    public Follower follower;
    private Timer pathTimer, opModeTimer;
    private int pathState;

    private boolean flyWheelOn;
    // ----- alliance -----
    private ALLIANCE currentAlliance = ALLIANCE.RED;
    private enum ALLIANCE{
        RED,
        BLUE
    }
    // ---- pose ----
    private Pose startPose = new Pose(122.09038238702202, 123.24136500579375, Math.toRadians(35));
    private Pose shootPose = new Pose(86.53186558516802, 86.66975666280418, Math.toRadians(44));
    private Pose c1 = new Pose(88.26882966396293, 65.2421784472769);
    private Pose c2 = new Pose(106.1923522595597,56.21320973348785);
    private Pose intake1 = new Pose(88.7079953650058, 88.23986095017378, Math.toRadians(180));
    private Pose c3 = new Pose(97.45017381228274, 65.729258400927);
    private Pose shootIntk1 = new Pose(131.38122827346464, 64.83893395133255, Math.toRadians(41));
    private Pose c4 = new Pose(105.20915411355736, 67.69814600231749);
    private Pose intake2 = new Pose(131.38122827346464, 64.83893395133255, Math.toRadians(210));
    private Pose c5 = new Pose(104.39339513325608,70.29200463499419);
    private Pose shootIntake2 = new Pose(91.75550405561992, 92.10428736964079, Math.toRadians(39));
    private Pose c6 = new Pose(104.66396292004633, 80.50115874855157);
    private Pose intake3 = new Pose(129.92699884125142, 83.90845886442641);
    private Pose shootIntake3 = new Pose(95.41135573580534, 94.98493626882967, Math.toRadians(37));
    private Pose leave = new Pose(120, 72, Math.toRadians(90));

    // ------ path chains ------
    private PathChain shootPath1(){
        return follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(
                        startPose.getHeading()
                        ,shootPose.getHeading()
                )
                .build();
    }
    private PathChain intakePath1(){
        return follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, c1, c2, intake1))
                .setConstantHeadingInterpolation(
                        intake1.getHeading()
                )
                .build();
    }

    private PathChain shootPath2(){
        return follower.pathBuilder()
                .addPath(new BezierCurve(intake1, c3, shootIntk1))
                .setLinearHeadingInterpolation(intake1.getHeading(), shootIntk1.getHeading())
                .build();
    }

    private PathChain intakePath2(){
        return follower.pathBuilder()
                .addPath(new BezierCurve(shootIntk1, c4, intake2))
                .setConstantHeadingInterpolation(intake2.getHeading())
                .build();
    }
    private PathChain shootPath3(){
        return follower.pathBuilder()
                .addPath(new BezierCurve(intake2, c5, shootIntake2))
                .setLinearHeadingInterpolation(intake2.getHeading(), shootIntake2.getHeading())
                .build();
    }

    private PathChain intakePath3(){
        return follower.pathBuilder()
                .addPath(new BezierCurve(shootIntake2, c6, intake3))
                .setTangentHeadingInterpolation().build();
    }

    private PathChain shootPath4(){
        return follower.pathBuilder()
                .addPath(new BezierLine(intake3, shootIntake3))
                .setLinearHeadingInterpolation(intake3.getHeading(), shootIntake3.getHeading())
                .build();
    }
    private PathChain endPath(){
        return follower.pathBuilder()
                .addPath(new BezierLine(shootIntake3, leave))
                .setLinearHeadingInterpolation(shootIntake3.getHeading(), leave.getHeading())
                .build();
    }
    private void Alliance(ALLIANCE alliance){
        if(alliance == ALLIANCE.BLUE){
            startPose = startPose.mirror();
            shootPose = shootPose.mirror();
            c1 = c1.mirror();
            c2 = c2.mirror();
            intake1 = intake1.mirror();
            c3 = c3.mirror();
            shootIntk1 = shootIntk1.mirror();
            c4 = c4.mirror();
            intake2 = intake2.mirror();
            c5 = c5.mirror();
            shootIntake2 = shootIntake2.mirror();
            c6 = c6.mirror();
            intake3 = intake3.mirror();
            shootIntake3 = shootIntake3.mirror();
        }
    }
    // ------- methods -------
    public void setPathState(int newState){
        pathState = newState;
        pathTimer.resetTimer();
    }
    public void statePathUpd() {
        switch (pathState){
            case 0:
                follower.followPath(shootPath1());
                setPathState(1);
                break;
            case 1:
                this.autoAllShoot();
                if (!follower.isBusy()){
                    follower.followPath(intakePath1(), true);
                    setPathState(2);
                }
                break;
            case 2:
                this.hardwareManager.intake(true);
                if (!follower.isBusy()){
                    follower.followPath(shootPath2(), true);
                    setPathState(3);
                }
                break;
            case 3:
                this.autoAllShoot();
                if(!follower.isBusy()){
                    follower.followPath(intakePath2(), true);
                    setPathState(4);
                }
                break;
            case 4:
                this.hardwareManager.intake(true);
                if(!follower.isBusy()){
                    follower.followPath(shootPath3(), true);
                    setPathState(5);
                }
                break;
            case 5:
                this.autoAllShoot();
                if(!follower.isBusy()){
                    follower.followPath(intakePath3(), true);
                    setPathState(6);
                }
                break;
            case 6:
                this.hardwareManager.intake(true);
                if(!follower.isBusy()){
                    follower.followPath(shootPath4(), true);
                    setPathState(7);
                }
                break;
            case 7:
                this.autoAllShoot();
                if (!follower.isBusy()){
                    follower.followPath(endPath(), true);
                    setPathState(8);
                }
                break;
            default:
                telemetry.addLine("No State");
                break;
        }
    }

    @Override
    public void init(){
        if(gamepad1.dpad_right){
            if(currentAlliance == ALLIANCE.RED){
                currentAlliance = ALLIANCE.BLUE;
                Alliance(currentAlliance);
            }else{
                currentAlliance = ALLIANCE.RED;
                Alliance(currentAlliance);
            }
        }
        pathTimer = new Timer();
        opModeTimer = new Timer();
        opModeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        telemetry.addData("Status", "Initialized");
    }
    @Override
    public void start(){
        opModeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop(){
        follower.update();
        statePathUpd();
        telemetry.addData("Alliance", currentAlliance);
        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.update();
        hardwareManager = new HardwareManagerNew(this);
        this.hardwareManager.positionFly(this.flyWheelOn);
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