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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Rabbot: Teleop POV", group="Rabbot")
//@Disabled
public class JCHSRabbotTeleopPOV_Linear extends LinearOpMode {

    /* Declare OpMode members. */
    JCHSHardwareRabbot rabbot = new JCHSHardwareRabbot();   // Use a Pushbot's hardware
    double             clawOffset      = 0;                       // Servo mid position
    final double       CLAW_SPEED      = 0.05;                   // sets rate to move servo

    @Override

    public void runOpMode() throws InterruptedException {
        double leftFront;
        double rightFront;
        double leftBack;
        double rightBack;
        double drive;
        double strafe;
        double turn=0;
        // double intake;
        double max;
        double clawHeight = 0;
        boolean sniping = false;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        rabbot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        rabbot.armMotor.setTargetPosition(0);
        rabbot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.
            if (gamepad1.dpad_down && sniping)
            {
                while (gamepad1.dpad_down)
                {
                    sniping = false;
                }
            }
            else if (gamepad1.dpad_down)
            {
                while (gamepad1.dpad_down)
                {
                    sniping = true;
                }
            }
            if (sniping)
            {
                drive = -gamepad1.left_stick_y/6;
            }
            else
            {
                drive = -gamepad1.left_stick_y;
            }
            if (sniping)
            {
                strafe = -gamepad1.right_stick_x/2;
            }
            else
            {
                strafe = -gamepad1.right_stick_x;
            }
            if (gamepad1.dpad_right)
            {
                if (sniping)
                {
                    turn=-0.15;
                }
                else
                {
                    turn=-0.8;
                }
            }
            else if (gamepad1.dpad_left)
            {
                if (sniping)
                {
                    turn=0.18;
                }
                else
                {
                    turn=0.5;
                }
            }
            else
            {
                turn = 0;
            }


            // Combine drive and turn for blended motion.
            leftFront  = drive + turn + strafe;
            leftBack  = drive + turn - strafe;
            rightFront = drive - turn - strafe;
            rightBack = drive - turn + strafe;


            // Normalize the values so neither exceed +/- 1.0
            //max = Math.max(Math.abs(leftFront), Math.abs(rightFront));
            max = Math.max(Math.max(Math.abs(leftFront), Math.abs(rightFront)), Math.max(Math.abs(leftBack), Math.abs(rightBack)));
            //max = drive + strafe + turn;
            if (max > 1.0)
            {
                leftFront /= max;
                rightFront /= max;
                leftBack /= max;
                rightBack /= max;
            }
            /*
            if (intake > 1.0)
            {
                intake = 1.0;
            }
            */

            // Output the safe vales to the motor drives.
            rabbot.leftDrive.setPower(leftFront);
            rabbot.rightDrive.setPower(-rightFront);
            rabbot.leftRamp.setPower(-leftBack);
            rabbot.rightRamp.setPower(rightBack);

            /*
            rabbot.Intake.setPower(intake);
            rabbot.crateHolder.setPower(intake);
             */

            // Use gamepad left & right Bumpers to open and close the claw
            if (gamepad1.right_bumper)
                //clawOffset += CLAW_SPEED;
                rabbot.clawServo.setPosition(1);
                //rabbot.clawServo.(1);
            else if (gamepad1.left_bumper)
                //clawOffset -= CLAW_SPEED;
                rabbot.clawServo.setPosition(-1);
            else
                rabbot.clawServo.setPosition(0.51);
            //rabbot.clawServo.setPosition(0.05);

            // Move both servos to new position.  Assume servos are mirror image of each other.
            //clawOffset = Range.clip(clawOffset, -0.30, 0.30);
            //rabbot.clawServo.setPosition(rabbot.MID_SERVO + clawOffset);
            //rabbot.leftClaw.setPosition(rabbot.MID_SERVO + clawOffset);
            //rabbot.rightClaw.setPosition(rabbot.MID_SERVO - clawOffset);
            //rabbot.clawServo.setPosition(rabbot.MID_SERVO + clawOffset);
            //rabbot.clawServo.setPosition(rabbot.MID_SERVO - clawOffset);

            // Use gamepad buttons to move arm up (Y) and down (A)
            //rabbot.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            while (gamepad1.right_trigger > 0)
            {
                rabbot.armMotor.setTargetPosition(2552);
                if (rabbot.armMotor.getCurrentPosition() < rabbot.armMotor.getTargetPosition())
                {
                    rabbot.armMotor.setVelocity(gamepad1.right_trigger * 2000);
                }
                rabbot.armMotor.setTargetPosition(rabbot.armMotor.getCurrentPosition());
            }
            while (gamepad1.left_trigger > 0)
            {
                rabbot.armMotor.setTargetPosition(0);
                if (rabbot.armMotor.getCurrentPosition() > rabbot.armMotor.getTargetPosition())
                {
                    rabbot.armMotor.setVelocity(gamepad1.left_trigger * -1300);
                }
                rabbot.armMotor.setTargetPosition(rabbot.armMotor.getCurrentPosition());
            }
                if (rabbot.armMotor.getCurrentPosition() < rabbot.armMotor.getTargetPosition())
                {
                    rabbot.armMotor.setVelocity(6000);
                }
                else
                {
                    rabbot.armMotor.setVelocity(0);
                }

            /*if (gamepad1.b)
            {
                // Set the motor's target position to 300 ticks
                rabbot.armMotor.setTargetPosition(1276);
                rabbot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                // Switch to RUN_TO_POSITION mode
                while (opModeIsActive() && rabbot.armMotor.getCurrentPosition() < rabbot.armMotor.getTargetPosition()) {
                    // Start the motor moving by setting the max velocity to 200 ticks per second
                    rabbot.armMotor.setVelocity(420);
                }
            }*/
            /*if (gamepad1.x)
            {
                // Set the motor's target position to 300 ticks
                rabbot.armMotor.setTargetPosition(-1260);
                // Switch to RUN_TO_POSITION mode
                rabbot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (opModeIsActive() && rabbot.armMotor.getCurrentPosition() > rabbot.armMotor.getTargetPosition()) {
                    // Start the motor moving by setting the max velocity to 200 ticks per second
                    rabbot.armMotor.setVelocity(300);
                }
            }*/



            // Send telemetry message to signify robot running;
            telemetry.addData("claw",  "Offset = %.2f", clawOffset);
            telemetry.addData("left",  "%.2f", leftFront);
            telemetry.addData("right", "%.2f", rightFront);
            telemetry.addData("leftBack",  "%.2f", leftBack);
            telemetry.addData("rightBack", "%.2f", rightBack);
            telemetry.addData("armTicks", rabbot.armMotor.getCurrentPosition());
            telemetry.addData("sniping", sniping);
            telemetry.update();
            //rabbot.armMotor.setTargetPosition(0);

            // Pace this loop so jaw action is reasonable speed.
            sleep(50);
        }
    }
}