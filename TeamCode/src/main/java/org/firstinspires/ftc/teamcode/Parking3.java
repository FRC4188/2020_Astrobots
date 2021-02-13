
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

//2.286
@Autonomous(name = "Parking3")
public class Parking3 extends LinearOpMode {

    private DcMotor frMotor, flMotor, brMotor, blMotor, intakeMotor, magazineMotor;
    private DcMotorEx shooterMotor;



    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        //
        waitForStart();
        drivetrain(0, 0,0.028);

        setAllPower(1.0);

        while (opModeIsActive() && isRobotBusy()) {
            //checkMotors();
            idle();
        }
        setAllPower(0.0);

        resetStartTime();


        //-------------------------------------------------------------
        drivetrain(2.8,0,0);
        setAllPower(1.0);

        while (opModeIsActive() && isRobotBusy()) {
            //checkMotors();
            idle();
        }
        setAllPower(0);

        //-------------------------------------------------------------
        drivetrain(0,0,-0.02);
        setAllPower(1.0);

        while (opModeIsActive() && isRobotBusy()) {
            //checkMotors();
            idle();
        }
        setAllPower(0);


        //--------------------------------------------------------------
        drivetrain(-1.8, 0,0);
        setAllPower(1.0);


        while (opModeIsActive() && isRobotBusy()) {
            //checkMotors();
            idle();
        }
        setAllPower(0.0);


        //-------------------------------------------------------------
        drivetrain(0, 0,-0.08);
        setAllPower(1.0);
        while (opModeIsActive() && isRobotBusy()) {
            //checkMotors();
            idle();
        }
        setAllPower(0.0);


        runtime.reset();

        while (opModeIsActive()){
            telemetry.addData("Velocity: ", shooterMotor.getVelocity());
            telemetry.addData("RunMode: ", shooterMotor.getMode());
            telemetry.update();
            double time = runtime.seconds();
            if (time > 0 && time < 10){
                shooterMotor.setVelocity(1180);
            }
            if (time > 10 && time < 14){
                magazineMotor.setPower(1.0);
                intakeMotor.setPower(-1.0);
            }
            if (time > 14 && time < 15){
                intakeMotor.setPower(0.0);
                shooterMotor.setPower(0.0);
                magazineMotor.setPower(0.0);
                shooterMotor.setVelocity(0);
            }
            if (time > 15 && time < 16){
                drivetrain(0.8, 0,0);
                setAllPower(1.0);

                while (opModeIsActive() && isRobotBusy()) {
                    //checkMotors();
                    idle();
                }
                setAllPower(0.0);
            }
        }



    }

    private void initialize() {

        frMotor = hardwareMap.get(DcMotor.class, "frMotor");
        flMotor = hardwareMap.get(DcMotor.class, "flMotor");
        brMotor = hardwareMap.get(DcMotor.class, "brMotor");
        blMotor = hardwareMap.get(DcMotor.class, "blMotor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        magazineMotor = hardwareMap.get(DcMotor.class, "magazineMotor");
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");


        frMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        brMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        blMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        magazineMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    private void setMotorMode(DcMotor.RunMode runMode) {
        frMotor.setMode(runMode);
        flMotor.setMode(runMode);
        brMotor.setMode(runMode);
        blMotor.setMode(runMode);
    }

    private void setAllPower(double power) {
        frMotor.setPower(power);
        flMotor.setPower(power);
        brMotor.setPower(power);
        blMotor.setPower(power);
    }
    private void checkMotors(){
        if(!frMotor.isBusy()) {
            frMotor.setPower(0);
        }

        if(!flMotor.isBusy()) {
            flMotor.setPower(0);
        }

        if(!brMotor.isBusy()) {
            brMotor.setPower(0);
        }

        if(!blMotor.isBusy()) {
            blMotor.setPower(0);
        }
    }

    private boolean isRobotBusy() {
        return blMotor.isBusy();
    }

    private void drivetrain(double forwardDistance, double strafeDistance, double rotationDistance) {


        frMotor.setTargetPosition(frMotor.getCurrentPosition() + (int) Math.round(Constants.TICKS_PER_METER * (forwardDistance - strafeDistance - rotationDistance)));
        flMotor.setTargetPosition(flMotor.getCurrentPosition() + (int) Math.round(Constants.TICKS_PER_METER * (forwardDistance + strafeDistance + rotationDistance)));
        brMotor.setTargetPosition(brMotor.getCurrentPosition() + (int) Math.round(Constants.TICKS_PER_METER * (forwardDistance + strafeDistance - rotationDistance)));
        blMotor.setTargetPosition(blMotor.getCurrentPosition() + (int) Math.round(Constants.TICKS_PER_METER * (forwardDistance - strafeDistance + rotationDistance)));

        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    private void driveSeconds(double forward, double strafe, double rotation) {
        frMotor.setPower(forward - strafe - rotation);
        flMotor.setPower(forward + strafe + rotation);
        brMotor.setPower(forward + strafe - rotation);
        blMotor.setPower(forward - strafe + rotation);
    }
    /*private void setLeftPower(double power, ) {
        frMotor.setTargetPosition((int) Math.round(-Constants.TICKS_PER_METER * 1.72));
        flMotor.setTargetPosition((int) Math.round(-Constants.TICKS_PER_METER * 1.72));
        brMotor.setTargetPosition((int) Math.round(-Constants.TICKS_PER_METER * 1.72));
        blMotor.setTargetPosition((int) Math.round(-Constants.TICKS_PER_METER * 1.72));
        frMotor.setPower(-power);
        flMotor.setPower(power);
        brMotor.setPower();
        blMotor.setPower(power);
    }
    private void setRightPower(double power) {
        frMotor.setPower(power);
        flMotor.setPower(0);
        brMotor.setPower(power);
        blMotor.setPower(0);
    }*/
}
