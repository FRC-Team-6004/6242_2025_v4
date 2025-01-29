// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.AutoCommands;

import frc.robot.util.LimelightHelpers;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxTagSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) *.5; // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.FieldCentric autodrive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driveStick = new CommandXboxController(0);
    
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final CommandSwerveDrivetrain autodrivetrain = TunerConstants.createDrivetrain();

    // Slew rate limiters to make driveStick inputs more gentle; 1/3 sec from 0 to 1.
    private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);
    //For sending auto to the dashboard
    private final SendableChooser<Command> autoChooser;
    
    public RobotContainer() {
        //commands for use in auto
        // Register Named Commands
        NamedCommands.registerCommand("coraltag", new InstantCommand(() -> driveToTag()));

        //smart dashboard for Auto paths
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);
        
        //swerve things
        configureBindings();

    }
    // simple proportional turning control with Limelight.
    // "proportional control" is a control algorithm in which the output is proportional to the error.
    // in this case, we are going to return an angular velocity that is proportional to the 
    // "tx" value from the Limelight.
    double limelight_aim_proportional()
    {    
        // kP (constant of proportionality)
        // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
        // if it is too high, the robot will oscillate.
        // if it is too low, the robot will never reach its target
        // if the robot never turns in the correct direction, kP should be inverted.
        double kP = .0175;

        // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
        // your limelight 3 feed, tx should return roughly 31 degrees.
        double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kP;

        // convert to radians per second for our drive method
        targetingAngularVelocity *= MaxAngularRate;

        //invert since tx is positive when the target is to the right of the crosshair
        targetingAngularVelocity *= -1.0;
        return targetingAngularVelocity;
    }

    // simple proportional ranging control with Limelight's "ty" value
    // this works best if your Limelight's mount height and target mount height are different.
    // if your limelight and target are mounted at the same or similar heights, use "ta" (area) for target ranging rather than "ty"
    double limelight_range_proportional()
    {    
        double kP = .1;
        double targetingForwardSpeed = LimelightHelpers.getTY("limelight") * kP;
        targetingForwardSpeed *= MaxTagSpeed;
        targetingForwardSpeed *= -1.0;
        System.out.println(targetingForwardSpeed);
        return targetingForwardSpeed;
    }

    private void configureBindings() {
        System.out.println("config bindings");
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-m_xspeedLimiter.calculate(driveStick.getLeftY()) * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-m_yspeedLimiter.calculate(driveStick.getLeftX()) * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-m_rotLimiter.calculate(driveStick.getRightX()) * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
            
        );

        driveStick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driveStick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driveStick.getLeftY(), -driveStick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driveStick.back().and(driveStick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driveStick.back().and(driveStick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driveStick.start().and(driveStick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driveStick.start().and(driveStick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        driveStick.rightBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        driveStick.y().whileTrue(new InstantCommand(() -> driveToTag()));
        driveStick.leftBumper().whileTrue(
                //lime light aim speaker                
                drivetrain.applyRequest(() -> drive.withVelocityX(limelight_range_proportional())
                .withVelocityY(-1 * MaxTagSpeed*(.4))
                .withRotationalRate(limelight_aim_proportional()))
        );

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private void driveToTag() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        System.out.println("drive to tag");
        double myX = 0;
        double myY = 0;
        double myTurn = 0;
        while (limelight_range_proportional() > .04 && limelight_range_proportional() != 0.0){

        
            autodrivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                
                autodrivetrain.applyRequest(() ->
                    autodrive.withVelocityX(limelight_range_proportional()) // Drive forward with negative Y (forward)
                        .withVelocityY(-1 * MaxTagSpeed*(.4)) // Drive left with negative X (left)
                        .withRotationalRate(limelight_aim_proportional()) // Drive counterclockwise with negative X (left)
                )
                
                
            );
        }            
           
        autodrivetrain.applyRequest(() ->
                    autodrive.withVelocityX(0) // Drive forward with negative Y (forward)
                        .withVelocityY(0) // Drive left with negative X (left)
                        .withRotationalRate(0) // Drive counterclockwise with negative X (left)
                )  ;
            
        
    }


    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
