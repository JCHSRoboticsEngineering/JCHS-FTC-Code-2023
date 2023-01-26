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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a modified Pushbot that we call Rabbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left front  drive motor:  "left_front_drive"
 * Motor channel:  Right front drive motor:  "right_front_drive"
 * Motor channel:  Left back drive motor:    "left_back_drive"
 * Motor channel:  Right back drive motor:   "right_back_drive"
 * Motor channel:  Intake motor:             "intake_wheel"
 * Motor channel:  Shooter motor:            "shooter_wheel"
 * Servo channel:  Wobble holding arm:       "wobble_arm"
 * Servo channel:  Wobble holding claw:      "wobble_claw"
 * Servo channel:  Servo to angle the ramp:  "shooter_angler"
 */

public class JCHSHardwareRabbot2021 extends JCHSHardwareRabbotChassis
{
    public DcMotor intakeWheel = null;
    public DcMotor shooterWheel = null;

    public Servo wobbleArm = null;
    public Servo wobbleClaw = null;
    public Servo shooterAngler = null;

  //  private ElapsedTime period  = new ElapsedTime();

    public static final double MID_SERVO       =  0.5 ;
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;

    /* Constructor */
    public JCHSHardwareRabbot2021()
    {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        super.init(ahwMap);
        
        intakeWheel = hwMap.get(DcMotor.class, "intake_wheel");
        shooterWheel = hwMap.get(DcMotor.class, "shooter_wheel");

        intakeWheel.setDirection(DcMotor.Direction.FORWARD); // TODO: Test the direction
        shooterWheel.setDirection(DcMotor.Direction.REVERSE);// TODO: Test the direction

        intakeWheel.setPower(0);
        shooterWheel.setPower(0);

        intakeWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        wobbleArm  = hwMap.get(Servo.class, "wobble_arm");
        wobbleClaw = hwMap.get(Servo.class, "wobble_claw");
        shooterAngler = hwMap.get(Servo.class, "shooter_angler");

        wobbleArm.setPosition(MID_SERVO);
        wobbleClaw.setPosition(MID_SERVO);
        shooterAngler.setPosition(MID_SERVO);
    }
 }