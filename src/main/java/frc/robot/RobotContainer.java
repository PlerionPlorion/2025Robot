package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.constVision;
import frc.robot.Constants.reefPosition;
import frc.robot.commands.AddVisionMeasurement;
import frc.robot.commands.TeleopElevatorInstant;
import frc.robot.commands.TeleopIntake;
import frc.robot.commands.TeleopOuttake;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.zeroing.ManualZeroElevator;
import frc.robot.commands.zeroing.ZeroElevator;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
        /* Subsystems */
        private final Swerve s_Swerve = new Swerve();
        public final Elevator elevator = new Elevator();
        public final Intake intake = new Intake();
        public final Limelight limelight = new Limelight();
        // public final AlgaeIntake algaeIntake = new AlgaeIntake();

        /* PathPlanner */
        @NotLogged
        SendableChooser<Command> autoChooser = new SendableChooser<>();

        /* Controllers */
        private final XboxController driver = new XboxController(0);

        /* Drive Controls */
        private final int translationAxis = XboxController.Axis.kLeftY.value;
        private final int strafeAxis = XboxController.Axis.kLeftX.value;
        private final int rotationAxis = XboxController.Axis.kRightX.value;

        /* Driver Buttons */
        private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kStart.value);
        private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftStick.value);
        public Trigger btn_LeftTrigger = new Trigger(() -> driver.getLeftTriggerAxis() > 0.7);
        public Trigger btn_RightTrigger = new Trigger(() -> driver.getRightTriggerAxis() > 0.7);
        // private final JoystickButton alignLButton = new JoystickButton(driver,
        // XboxController.Button.kLeftBumper.value); // Fix to Left Num
        // private final JoystickButton alignRButton = new JoystickButton(driver,
        // XboxController.Button.kRightBumper.value); // Fix to Right Num
        // private final JoystickButton cleanL2Button = new JoystickButton(driver,
        // XboxController.Button.kRightBumper.value);
        // private final JoystickButton cleanL3Button = new JoystickButton(driver,
        // XboxController.Button.kLeftBumper.value);
        // private final JoystickButton algaeZero = new JoystickButton(driver,
        // XboxController.Button.kB.value);
        private final JoystickButton outtakeButton = new JoystickButton(driver,
                        XboxController.Button.kA.value);
        private final JoystickButton intakeButton = new JoystickButton(driver,
                        XboxController.Button.kX.value);
        private final JoystickButton zeroSubsystem = new JoystickButton(driver,
                        XboxController.Button.kY.value);
        // private final JoystickButton zeroAlgae = new JoystickButton(driver,
        // XboxController.Button.kBack.value);
        private final POVButton L4 = new POVButton(driver, Constants.POV_UP);
        private final POVButton L3 = new POVButton(driver, Constants.POV_LEFT);
        private final POVButton L2 = new POVButton(driver, Constants.POV_RIGHT);
        private final POVButton NONE = new POVButton(driver, Constants.POV_DOWN);

        Command manualZeroSubsystems = new ManualZeroElevator(elevator)
                        .ignoringDisable(true).withName("ManualZeroSubsystems");

        public Command AddVisionMeasurement() {
                return new AddVisionMeasurement(s_Swerve, limelight)
                                .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)
                                .ignoringDisable(true);
        }

        public void setMegaTag2(boolean setMegaTag2) {

                if (setMegaTag2) {
                        s_Swerve.swervePoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(
                                        constVision.MEGA_TAG2_STD_DEVS_POSITION,
                                        constVision.MEGA_TAG2_STD_DEVS_POSITION,
                                        constVision.MEGA_TAG2_STD_DEVS_HEADING));
                } else {
                        // Use MegaTag 1
                        s_Swerve.swervePoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(
                                        constVision.MEGA_TAG1_STD_DEVS_POSITION,
                                        constVision.MEGA_TAG1_STD_DEVS_POSITION,
                                        constVision.MEGA_TAG1_STD_DEVS_HEADING));
                }
                limelight.setMegaTag2(setMegaTag2);
        }

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                RobotController.setBrownoutVoltage(5.5);
                configureAutoBindings();
                s_Swerve.setDefaultCommand(
                                new TeleopSwerve(
                                                s_Swerve,
                                                elevator,
                                                () -> -driver.getRawAxis(translationAxis),
                                                () -> -driver.getRawAxis(strafeAxis),
                                                () -> -driver.getRawAxis(rotationAxis),
                                                () -> robotCentric.getAsBoolean(),
                                                () -> btn_LeftTrigger.getAsBoolean(),
                                                () -> btn_RightTrigger.getAsBoolean()));
                // SmartDashboard.putData("Auto Chooser", autoChooser);
                // Configure the button bindings
                configureButtonBindings();
                configureAutoSelector();
        }

        /**
         * Use this method to define your button->command mappings. Buttons can be
         * created by
         * instantiating a {@link GenericHID} or one of its subclasses ({@link
         * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
         * it to a {@link
         * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
         */

        private void configureButtonBindings() {
                /* Driver Buttons */
                zeroGyro.onTrue(new InstantCommand(
                                () -> s_Swerve.resetOdometry(Constants.constField.getFieldPositions().get()[0])));
                // alignRButton.whileTrue(new TeleopLimelightDrive(s_Swerve, limelight, true));
                // alignLButton.whileTrue(new TeleopLimelightDrive(s_Swerve, limelight, false));
                // extendElevator.onTrue(new TeleopElevator(elevator, intake, false)
                // .withInterruptBehavior(InterruptionBehavior.kCancelSelf));
                // retractElevator.onTrue(new TeleopElevator(elevator, intake, true)
                // .withInterruptBehavior(InterruptionBehavior.kCancelSelf));
                zeroSubsystem.onTrue(new ParallelCommandGroup(new ZeroElevator(elevator)
                                .withTimeout(Constants.constElevator.ZEROING_TIMEOUT.in(Units.Seconds))));
                // zeroAlgae.onTrue(new ZeroAlgaeIntake(algaeIntake)
                // .withTimeout(constAlgaeIntake.ZEROING_TIMEOUT.in(Units.Seconds)));
                outtakeButton.whileTrue(new TeleopOuttake(intake));
                intakeButton.whileTrue(new TeleopIntake(intake));
                // cleanL2Button.onTrue(new CleanL2Reef(elevator, algaeIntake));
                // algaeZero.onTrue(new AlgaeZero(elevator, algaeIntake));
                L4.onTrue(new TeleopElevatorInstant(elevator, intake, reefPosition.L4));
                L3.onTrue(new TeleopElevatorInstant(elevator, intake, reefPosition.L3));
                L2.onTrue(new TeleopElevatorInstant(elevator, intake, reefPosition.L2));
                NONE.onTrue(new TeleopElevatorInstant(elevator, intake, reefPosition.NONE));
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
                // try{
                // PathPlannerPath path = PathPlannerPath.fromPathFile("2025");
                // return AutoBuilder.followPath(path);
                // } catch (Exception e) {
                // return Commands.none();
                // }
                // return null;
        }

        public Command getWarmUpCommand() {
                return AutoBuilder.buildAuto("Taxi");
                // try{
                // PathPlannerPath path = PathPlannerPath.fromPathFile("2025");
                // return AutoBuilder.followPath(path);
                // } catch (Exception e) {
                // return Commands.none();
                // }
                // return null;
        }

        private void configureAutoSelector() {
                autoChooser = AutoBuilder.buildAutoChooser("Taxi");
                SmartDashboard.putData("Auto Chooser", autoChooser);
        }

        private void configureAutoBindings() {

                Command driveAutoAlignLeft = Commands.runOnce(() -> s_Swerve.autoAlign(Meters.of(0),
                                s_Swerve.getDesiredReef(true), MetersPerSecond.of(0),
                                MetersPerSecond.of(0), DegreesPerSecond.of(0), 1.0, false, Meters.of(1000)))
                                .repeatedly();

                Command intakeAuto = new SequentialCommandGroup(
                                new InstantCommand(() -> intake.setVoltage(Constants.constIntake.INTAKE_VOLTAGE))
                                                .withTimeout(3))
                                .andThen(new InstantCommand(() -> intake.setVoltage(0)));

                NamedCommands.registerCommand("Intake", new TeleopIntake(intake).withTimeout(2));
                NamedCommands.registerCommand("Place Coral", new TeleopOuttake(intake).withTimeout(0.5));

                NamedCommands.registerCommand("PlaceSequence",
                                Commands.sequence(
                                                driveAutoAlignLeft.asProxy().until(() -> s_Swerve.isAligned())
                                                                .withTimeout(1)
                                                                .andThen(new ParallelCommandGroup(new InstantCommand(
                                                                                () -> LimelightHelpers
                                                                                                .setLEDMode_ForceOff(
                                                                                                                Constants.constVision.LIMELIGHT_NAMES[0])),
                                                                                new InstantCommand(
                                                                                                () -> LimelightHelpers
                                                                                                                .setLEDMode_ForceOff(
                                                                                                                                Constants.constVision.LIMELIGHT_NAMES[1])))),
                                                Commands.runOnce(
                                                                () -> s_Swerve.autoDrive(new ChassisSpeeds(), false))));

                NamedCommands.registerCommand("zeroElevator",
                                new TeleopElevatorInstant(elevator, intake, reefPosition.NONE).withTimeout(1.5));
                NamedCommands.registerCommand("PrepL4",
                                new TeleopElevatorInstant(elevator, intake, reefPosition.L4).withTimeout(1.5));

                // EventTrigger prepL4 = new EventTrigger("PrepL4");
                // prepL4.onTrue(new TeleopElevatorInstant(elevator, intake,
                // reefPosition.L4).withTimeout(1.5));

                // EventTrigger zeroElevator = new EventTrigger("zeroElevator");
                // zeroElevator.onTrue(new TeleopElevatorInstant(elevator, intake,
                // reefPosition.NONE).withTimeout(1.5));
        }

        public void resetToAutoPose() {
                Rotation2d desiredRotation = Rotation2d.kZero;

                try {
                        desiredRotation = PathPlannerAuto.getPathGroupFromAutoFile(autoChooser.getSelected().getName())
                                        .get(0)
                                        .getIdealStartingState().rotation();
                } catch (Exception e) {
                        // System.out.println("badAutoRot");
                }

                s_Swerve.resetOdometry(new Pose2d(s_Swerve.getPoseEstimator().getTranslation(), desiredRotation));
        }
}
