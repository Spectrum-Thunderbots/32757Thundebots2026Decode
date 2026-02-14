package org.firstinspires.ftc.teamcode.main;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name="Main Linear Autonomous New")
public class MainAutoNew extends LinearOpMode {

    private static final String VERSION = "2025.12.13.3";
    private static final String INFO = "Main Auto V"+VERSION;
    private boolean redAlliance;
    private boolean closePosition;
    private int balls;

    private boolean fly;

    private HardwareManagerNew hardwareManager = null;

    @Override
    public void runOpMode() throws InterruptedException {
        this.telemetry.addData("Status", "Initializing");
        this.telemetry.addData("Info", INFO);
        this.telemetry.update();


        this.hardwareManager = new HardwareManagerNew(this);
        this.hardwareManager.init(true);

        this.telemetry.addData("Status", "Initializing - Action required");
        this.telemetry.addData("Info", INFO);
        this.telemetry.addData("Important", "Please select alliance");
        this.telemetry.addData("Y", "Red");
        this.telemetry.addData("A", "Blue");
        this.telemetry.update();


        while (!(this.gamepad1.yWasPressed() || this.gamepad1.aWasPressed() || this.isStopRequested())) {}
        if (this.isStopRequested()) return;
        this.redAlliance = this.gamepad1.y;

        this.telemetry.addData("Status", "Initializing - Action required");
        this.telemetry.addData("Info", INFO);
        this.telemetry.addData("Important", "Please select position");
        this.telemetry.addData("Y", "Close position");
        this.telemetry.addData("A", "Far Position");
        this.telemetry.update();

        while (!(this.gamepad1.yWasPressed() || this.gamepad1.aWasPressed() || this.isStopRequested())) {}
        if (this.isStopRequested()) return;
        this.closePosition = this.gamepad1.y;

        this.telemetry.addData("Status", "Initializing - Action required");
        this.telemetry.addData("Info", INFO);
        this.telemetry.addData("Important", "Please select a goal");
        this.telemetry.addData("Y", "9 ball");
        this.telemetry.addData("X", "6 balls");
        this.telemetry.addData("A", "3 balls");
        this.telemetry.update();

        while (!(this.gamepad1.yWasPressed() || this.gamepad1.xWasPressed() || this.gamepad1.aWasPressed() || this.isStopRequested())) {}
        if (this.isStopRequested()) return;
        this.balls = this.gamepad1.y ? 9 : (this.gamepad1.x ? 6 : 3);


        this.telemetry.addData("Status", "Initialized");
        this.telemetry.addData("Info", INFO);
        this.telemetry.addData("Mode", (this.redAlliance ? "Red" : "Blue") + "Alliance;" + (this.closePosition ? "Close" : "Far") + "Position;" + this.balls + " balls");
        this.telemetry.update();

        this.hardwareManager.resetOdo();
        this.hardwareManager.getAlliance(redAlliance);

        //telemetry.addData("Robot Angle", this.hardwareManager.getIMU(AngleUnit.DEGREES));
        this.telemetry.update();

        waitForStart();


        if (!this.redAlliance && !this.closePosition){
            prepShootFar();
            turn(-7.5);
            autoAllShoot();
            turn(7.5);
            if (this.balls > 3) {
                drive(28);
                turn(27);
                intake(true);
                drive(-64);
                drive(64);
                intake(false);
                turn(-27);
                prepShootFar();
                drive(-28);
                turn(-7.5);
                autoAllShoot();
                turn(7);
                if (this.balls > 6){
                    drive(64);
                    turn(27);
                    intake(true);
                    drive(-64);
                    drive(64);
                    intake(false);
                    turn(-27);
                    prepShootFar();
                    drive(-64);
                    turn(-7.5);
                    autoAllShoot();
                    turn(7.5);
                }
            }
            drive(45);
            turn(-28);
        }
        if (this.redAlliance && !this.closePosition){
            prepShootFar();
            turn(7.5);
            autoAllShoot();
            turn(-7.5);
            if (balls > 3){
                drive(28);
                turn(-27);
                intake(true);
                drive(-64);
                intake(false);
                drive(64);
                turn(27);
                prepShootFar();
                drive(-28);
                turn(7.5);
                autoAllShoot();
                turn(-7);
                if (balls > 6){
                    drive(54);
                    turn(-27);
                    intake(true);
                    drive(-64);
                    intake(false);
                    drive(64);
                    turn(27);
                    prepShootFar();
                    drive(-54);
                    turn(7.5);
                    autoAllShoot();
                    turn(-7.5);
                }
            }
            drive(45);
            turn(28);
        }
        if (!this.redAlliance && this.closePosition) {
            prepShootClose();
            drive(-66);
            autoAllShoot();
            if (balls > 3){
                turn(42);
                intake(true);
                drive(-52);
                prepShootClose();
                drive(52);
                intake(false);
                turn(-42);
                autoAllShoot();
                if (balls > 6){
                    turn(15);
                    drive(-36);
                    turn( 27);
                    intake(true);
                    drive(-64);
                    intake(false);
                    prepShootClose();
                    drive(64);
                    turn(-27);
                    drive(36);
                    turn(-15);
                    autoAllShoot();
                }
            }
            turn(15);
            drive(-36);
            turn(-28);
        }
        if (this.redAlliance && this.closePosition) {
            prepShootClose();
            drive(-66);
            autoAllShoot();
            if (balls > 3){
                turn(-42);
                intake(true);
                drive(-52);
                prepShootClose();
                drive(52);
                intake(false);
                turn(42);
                autoAllShoot();
                if (balls > 6){
                    turn(-15);
                    drive(-36);
                    turn(-27);
                    intake(true);
                    drive(-64);
                    intake(false);
                    prepShootClose();
                    drive(64);
                    turn(27);
                    drive(36);
                    turn(15);
                    autoAllShoot();
                }
            }
            turn(-15);
            drive(-36);
            turn(28);
        }
    }


    @Deprecated private void prepShootClose(){
        prepShoot();
    }

    @Deprecated private void prepShootFar(){
        prepShoot();
    }

    private void prepShoot(){
        this.fly = true;
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

        this.fly = false;
        this.hardwareManager.fly(false);
    }

    private void turn(double inches){
        if (this.fly) this.hardwareManager.encoderTurnDrive(inches, this.closePosition);
        else this.hardwareManager.encoderTurnDrive(inches);
    }

    private void drive(double inches){
        if (this.fly) this.hardwareManager.encoderForwardDrive(inches, this.closePosition);
        else this.hardwareManager.encoderForwardDrive(inches);
    }

    private void intake(boolean active){
        if (!active) intakeWait();
        this.hardwareManager.intake(active);
    }

    private void intakeWait(){
        this.hardwareManager.stopDrive();
        this.sleep(350);
    }
}
