
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

//2.286
@Autonomous(name = "Parking")
public class Parking extends LinearOpMode {

    private DcMotor frMotor, flMotor, brMotor, blMotor, intakeMotor, magazineMotor, verticalArm;
    private DcMotorEx shooterMotor;
    private CRServo horizontalArm;


    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        //
        waitForStart();

        horizontalArm.setPower(-1.0);

        drivetrain(1.4, 0,0);
        setAllPower(1.0);

        while (opModeIsActive() && isRobotBusy()) {
            //checkMotors();
            idle();
        }
        setAllPower(0.0);

        resetStartTime();

        //-------------------------------------------------------------

        drivetrain(0, 0,-0.03);
        setAllPower(1.0);

        while (opModeIsActive() && isRobotBusy()) {
            //checkMotors();
            idle();
        }
        setAllPower(0.0);

        resetStartTime();

        //-------------------------------------------------------------



        runtime.reset();

        while (opModeIsActive()){
            double time = runtime.seconds();
            if (time > 0 && time < 10){
                shooterMotor.setVelocity(1300);
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
                drivetrain(0, 0,-0.2);
                setAllPower(1.0);

                while (opModeIsActive() && isRobotBusy()) {
                    //checkMotors();
                    idle();
                }
                setAllPower(0.0);
            }
            if (time > 16 && time < 18){
                verticalArm.setPower(-0.5);
            }
            if (time > 18 && time < 19.5){
                verticalArm.setPower(0);
                horizontalArm.setPower(1.0);
            }
            if (time > 19.5 && time < 20){
                drivetrain(0.3, 0,0);
                setAllPower(1.0);

                while (opModeIsActive() && isRobotBusy()) {
                    //checkMotors();
                    idle();
                }
                setAllPower(0.0);
            }

            if (time > 20 && time < 20.5){
                drivetrain(0, 0,0.1);
                setAllPower(1.0);

                while (opModeIsActive() && isRobotBusy()) {
                    //checkMotors();
                    idle();
                }
                setAllPower(0.0);
            }

            if (time > 20.5 && time < 21){
                drivetrain(0, 0,0.1);
                setAllPower(1.0);

                while (opModeIsActive() && isRobotBusy()) {
                    //checkMotors();
                    idle();
                }
                setAllPower(0.0);

                break;
            }

            if (time > 21 && time < 21.5){
                drivetrain(0.5, 0,0);
                setAllPower(1.0);

                while (opModeIsActive() && isRobotBusy()) {
                    //checkMotors();
                    idle();
                }
                setAllPower(0.0);

                break;
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

        horizontalArm = hardwareMap.get(CRServo.class, "hArm");
        verticalArm = hardwareMap.get(DcMotor.class, "vArm");
        verticalArm.setDirection(DcMotorSimple.Direction.REVERSE);
        verticalArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



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

    /**
     * Increment a value by delta and return the new value.
     *
     * @param  forwardDistance   the distance, in meters, the robot should move forward
     * @param  strafeDistance    the distance, in meters, the robot should strafe sideways
     * @param  rotationDistance  the angle to rotate the robot
     */
    private void drivetrain(double forwardDistance, double strafeDistance, double rotationDistance) {

        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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

}
