package org.firstinspires.ftc.teamcode.extras;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import java.util.function.Supplier;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.main.HardwareManagerNew;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * The Main Iterative TeleOp for Thunderbots Too 2025
 * More documentation needs to be added in the future
 * @see HardwareManagerNew
 * @version 2026.01.07.0
 * @author Max & Devon
 */
@TeleOp(name = "Blue Main Iterative TeleOp New Blue", group = "iterative")
public class MainTeleOpBLUE extends OpMode {

    private static final String VERSION = "2025.12.13.0";

    //The HardwareManager instance for physical Robot interactions
    private HardwareManagerNew hardwareManager = null;

    //Stores robot states
    private boolean flyWheelOn = false;
    private boolean close = false;
    boolean redAlliance = false;

    //Pedro
    private boolean drive = true;
    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private final Pose farRedShoot = new Pose(88,10,Math.toRadians(69));
    private final Pose farBlueShoot = new Pose(56,10,Math.toRadians(114));
    private final Pose closeRedShoot = new Pose(85,85,Math.toRadians(39));
    private final Pose closeBlueShoot = new Pose(88,85,Math.toRadians(129));
    private Pose pose = (!close ? farBlueShoot : closeBlueShoot);

    //Stores the time running for further reference, eg telemetry
    private ElapsedTime runtime = null;


    private ElapsedTime shootTime = null;


    private int ballsToShoot = 0;

    @Override
    public void init() {
        this.telemetry.addData("Status", "initializing");
        this.telemetry.update();

        hardwareManager = new HardwareManagerNew(this);

        hardwareManager.init();
        this.flyWheelOn = false;
        this.ballsToShoot = 0;
        this.hardwareManager.odometryInitBlue();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

         pathChain = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, pose)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, pose.getHeading(), 0.8))
                .build();
    }

    @Override
    public void init_loop() {
        this.telemetry.addData("Status", "initialized");
        this.telemetry.addData("Info", "Main Iterative TeleOp V"+ VERSION +"; H"+HardwareManagerNew.VERSION);
        this.telemetry.update();
    }

    @Override
    public void start() {
        this.telemetry.addData("Status", "starting");
        this.telemetry.update();
        follower.startTeleOpDrive();

        this.runtime = new ElapsedTime();
    }

    @Override
    public void loop() {
        //Emergency break
        if (gamepad1.back) this.terminateOpModeNow();


        this.telemetry.addData("Status", "running for " + this.runtime.seconds() + " seconds");
        this.telemetry.addData("Info", "Main Iterative TeleOp V" + VERSION + "; H" + HardwareManagerNew.VERSION);


       // if (gamepad1.dpadLeftWasPressed()) redAlliance = !redAlliance;

        this.telemetry.addData("Drive Mode", drive ? "Field Relative" : "Pedro");
        if (drive){
            this.hardwareManager.driveFieldRelativeBlue(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        }
        if (gamepad1.aWasPressed()){
            follower.followPath(pathChain.get());
            drive = false;
        }
        if (!drive && (gamepad1.bWasPressed() || !follower.isBusy())) {
            drive = true;
        }

        if (gamepad1.rightBumperWasPressed()) this.flyWheelOn = !this.flyWheelOn;

        if (gamepad1.dpadDownWasPressed()) this.close = !close;

        this.telemetry.addData("Position", close ? "Close" : "Far");

        boolean outake = gamepad1.left_bumper;
        boolean intake = gamepad1.left_trigger > 0.1;

        hardwareManager.manualShoot(gamepad1.right_trigger > 0.2);

        if (gamepad1.xWasPressed()){
            this.hardwareManager.imuReset();
        }

        if (gamepad1.dpadRightWasPressed()){
            this.hardwareManager.resetOdoBlue();
        }

        boolean active = gamepad1.dpadUpWasPressed();

        this.hardwareManager.odometry.update();

        this.hardwareManager.setRobotPosition();

        this.telemetry.addData("Alliance", hardwareManager.getAlliance(true) ? "red" : "blue");

        this.hardwareManager.driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        this.hardwareManager.axialLateralYaw(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        this.hardwareManager.fly(this.flyWheelOn, close);
        this.hardwareManager.getPosition(close);
        if (intake || outake) hardwareManager.intake(true, intake);
        else hardwareManager.intake(false);
        this.hardwareManager.cameraStream(active);

        this.hardwareManager.getBlockPos();
        this.hardwareManager.odometryTelemetry();
        telemetry.addData("Robot Angle", this.hardwareManager.getIMU(AngleUnit.DEGREES));
        this.hardwareManager.driveTelemetry();
        this.hardwareManager.telemetryAprilTag();

    }

    @Override
    public void stop() {
        this.telemetry.addData("Status", "stopping after " + ((this.runtime != null) ? this.runtime.seconds() : 0) + " seconds");
    }
}
