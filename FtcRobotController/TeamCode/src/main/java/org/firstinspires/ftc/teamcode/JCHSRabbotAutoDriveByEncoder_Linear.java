/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Rabbot: Auto Drive By Encoder", group="Rabbot")
// @Disabled
public class JCHSRabbotAutoDriveByEncoder_Linear extends JCHSRabbotAutonomous {
    //Hi Alex

    /* Declare OpMode members. */
    JCHSHardwareRabbot      rabbot = new JCHSHardwareRabbot();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 28;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 4;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.75 ;     // For figuring circumference
    static final double     FUDGE_FACTOR            = 1.0;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION * FUDGE_FACTOR) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        rabbot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        rabbot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rabbot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rabbot.rightRamp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rabbot.leftRamp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //rabbot.crateHolder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //rabbot.Intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rabbot.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //rabbot.flyWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rabbot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rabbot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rabbot.rightRamp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rabbot.leftRamp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //rabbot.Intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //rabbot.crateHolder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rabbot.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //rabbot.flyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                rabbot.leftDrive.getCurrentPosition(),
                rabbot.rightDrive.getCurrentPosition());

        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        //encoderDrive(DRIVE_SPEED,  48,  48, 10, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
        //encoderDrive(TURN_SPEED,   12, -12, 10, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        //encoderDrive(DRIVE_SPEED, -24, -24, 10, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout

        //rabbot.leftClaw.setPosition(1.0);            // S4: Stop and close the claw.
        //rabbot.rightClaw.setPosition(0.0);

        sleep(1000);     // pause for servos to move

        telemetry.addData("Path", "Complete");
        telemetry.update();
        //encoderDrive(0.25,
         //       15, 0, 0,
         //       30);


    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public double getRunTime(){
        return runtime.seconds();
    }
    public void encoderDrive(double speed,
                             double xInches, double yInches, double armInches,
                             double timeoutS) {
        double newLeftTarget;
        double newRightTarget;
        double newLeftBackTarget;
        double newRightBackTarget;
        double newArmInches;
        double leftFront;
        double rightFront;
        double rightBack;
        double leftBack;
        double max;
        double drive = 0;
        double turn = 0;
        double strafe = 0;
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            drive = yInches;
            strafe = -xInches;
            leftFront  = drive + turn + strafe;
            leftBack  = drive + turn - strafe;
            rightFront = drive - turn - strafe;
            rightBack = drive - turn + strafe;


            // Normalize the values so neither exceed +/- 1.0
            //max = Math.max(Math.abs(leftFront), Math.abs(rightFront));
            //max = Math.max(Math.max(Math.abs(leftFront), Math.abs(rightFront)), Math.max(Math.abs(leftBack), Math.abs(rightBack)));
            //max = drive + strafe + turn;
            /*
            if (intake > 1.0)
            {
                intake = 1.0;
            }
            */
            // Determine new target position, and pass to motor controller
            newLeftTarget = rabbot.leftDrive.getCurrentPosition() + (leftFront * COUNTS_PER_INCH);
            newRightTarget = rabbot.rightDrive.getCurrentPosition() + (rightFront * COUNTS_PER_INCH);
            newLeftBackTarget = rabbot.leftRamp.getCurrentPosition() + (leftBack * COUNTS_PER_INCH);
            newRightBackTarget = rabbot.rightRamp.getCurrentPosition() + (rightBack * COUNTS_PER_INCH);
            newArmInches = rabbot.rightDrive.getCurrentPosition() + (armInches * COUNTS_PER_INCH);
            rabbot.leftRamp.setTargetPosition((int) Math.round(-newLeftBackTarget));
            rabbot.rightRamp.setTargetPosition((int) Math.round(newRightBackTarget));
            rabbot.leftDrive.setTargetPosition((int) (Math.round(newLeftTarget)));
            rabbot.rightDrive.setTargetPosition((int)(Math.round(-newRightTarget)));
            rabbot.armMotor.setTargetPosition((int)(Math.round(newArmInches)));


            // Turn On RUN_TO_POSITION
            rabbot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rabbot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rabbot.leftRamp.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rabbot.rightRamp.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //rabbot.Intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //rabbot.crateHolder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //rabbot.flyWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rabbot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            rabbot.leftDrive.setPower(Math.abs(speed));
            rabbot.rightDrive.setPower(Math.abs(speed));
            rabbot.leftRamp.setPower(Math.abs(speed));
            rabbot.rightRamp.setPower(Math.abs(speed));
            //rabbot.Intake.setPower(Math.abs(speed));
            //rabbot.crateHolder.setPower(Math.abs(speed));
            //rabbot.flyWheel.setPower(Math.abs(speed));
            rabbot.armMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (rabbot.leftDrive.isBusy() && rabbot.rightDrive.isBusy())) {

                rabbot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rabbot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rabbot.leftRamp.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rabbot.rightRamp.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //rabbot.Intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //rabbot.crateHolder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //rabbot.flyWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rabbot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                // Display it for the driver.
                //telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                //telemetry.addData("Path2",  "Running at %7d :%7d",
                        rabbot.leftDrive.getCurrentPosition();
                        rabbot.rightDrive.getCurrentPosition();
                        /*
                        //rabbot.leftFrontDrive.getCurrentPosition(),
                        //rabbot.rightFrontDrive.getCurrentPosition());
                        //rabbot.leftIntake.getCurrentPosition(),
                        //rabbot.rightIntake.getCurrentPosition(),
                        //rabbot.leftFrontDrive.getCurrentPosition(),
                        //rabbot.rightFrontDrive.getCurrentPosition());
                         */
                //telemetry.update();
            }

            // Stop all motion;
            rabbot.leftDrive.setPower(0);
            rabbot.rightDrive.setPower(0);
            rabbot.leftRamp.setPower(0);
            rabbot.rightRamp.setPower(0);
            //rabbot.Intake.setPower(0);
            //rabbot.crateHolder.setPower(0);
            //rabbot.flyWheel.setPower(0);
            rabbot.armMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            rabbot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rabbot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rabbot.leftRamp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rabbot.rightRamp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //rabbot.Intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //rabbot.crateHolder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //rabbot.flyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rabbot.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }


    }
}