package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Mat;
import org.opencv.core.Point;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants.ArmPosition;
import frc.robot.subsystems.*;

public class PathGPT extends CommandBase {

    private Swerve s_Swerve;
    private ArmSubsystem s_ArmSubsystem;
    private IntakeSubsystem s_IntakeSubsystem;
    private Limelight s_LimelightSubsystem;
    private int pieceNum;
    private boolean chargeStation;
    private Point startPos;
    private int phase = 0;
    private int piecesScored = 0;
    private int scoringNode = 0;
    private Point nodePos = null;
    private Point closestPiece = null;
    private List <Point> availablePieces = new ArrayList<Point>();
    private List <Point> highTraversalPoints = new ArrayList<Point>();
    private List <Point> lowTraversalPoints = new ArrayList<Point>();
    private List <Point> availableNodes = new ArrayList<Point>();
    private List <Point> chargeClimbPoints = new ArrayList<Point>();

    public PathGPT(Swerve s_Swerve, ArmSubsystem s_ArmSubsystem, 
                    Limelight s_LimelightSubsystem, IntakeSubsystem s_IntakeSubsystem, 
                    int pieceNum, boolean chargeStation, Point startPos){
        //addRequirements(s_Swerve);
        this.s_Swerve = s_Swerve;
        this.s_ArmSubsystem = s_ArmSubsystem;
        this.s_IntakeSubsystem = s_IntakeSubsystem;
        this.s_LimelightSubsystem = s_LimelightSubsystem;
        this.pieceNum = pieceNum;
        this.chargeStation = chargeStation;
        this.startPos = startPos;
        availablePieces.add(Constants.FieldConstants.g1);
        availablePieces.add(Constants.FieldConstants.g2);
        availablePieces.add(Constants.FieldConstants.g3);
        availablePieces.add(Constants.FieldConstants.g4);
        highTraversalPoints.add(Constants.FieldConstants.highXHighYTraversal);
        highTraversalPoints.add(Constants.FieldConstants.highXLowYTraversal);
        lowTraversalPoints.add(Constants.FieldConstants.lowXHighYTraversal);
        lowTraversalPoints.add(Constants.FieldConstants.lowXLowYTraversal);
        availableNodes.add(Constants.FieldConstants.n1);
        availableNodes.add(Constants.FieldConstants.n2);
        availableNodes.add(Constants.FieldConstants.n3);
        availableNodes.add(Constants.FieldConstants.n4);
        availableNodes.add(Constants.FieldConstants.n5);
        availableNodes.add(Constants.FieldConstants.n6);
        availableNodes.add(Constants.FieldConstants.n7);
        availableNodes.add(Constants.FieldConstants.n8);
        availableNodes.add(Constants.FieldConstants.n9);
        chargeClimbPoints.add(Constants.FieldConstants.highXClimb);
        chargeClimbPoints.add(Constants.FieldConstants.lowXClimb);
        closestNode(startPos);
    }

    public SequentialCommandWrapper generatePickupArmSequence () {

        List <CommandBase> commands = new ArrayList<>();
        List <SubsystemBase> subsystems = new ArrayList<>();

        ArmPosition floor = Constants.AutoConstants.ArmPosition.Floor;
        ArmPosition floorSequence = Constants.AutoConstants.ArmPosition.FloorSequence;

        commands.add(new PositionArm(s_ArmSubsystem, List.of(floorSequence, floor)));
        commands.add(new IntakeAuto(s_IntakeSubsystem, Constants.AutoConstants.intakeInAutoConstant));

        subsystems.add(s_IntakeSubsystem);
        subsystems.add(s_ArmSubsystem);

        return new SequentialCommandWrapper(subsystems, commands);
    }

    public void generatePickupSequence(SequentialCommandWrapper armSequence, PathPlannerTrajectory trajectory) {
        CommandBase driveCommand = s_Swerve.followTrajectoryCommand(trajectory, true).alongWith(armSequence).withTimeout(4);
        CommandBase notifyCommand = new InstantCommand(() -> lastPickupSequenceFinished());
        CommandBase zeroGyro = new InstantCommand(() -> s_Swerve.gyro180());

        List <CommandBase> commandList = new ArrayList<>();
        List <SubsystemBase> subsystems = new ArrayList<>();

        if (piecesScored == 0) {
            commandList.add(zeroGyro);
        }

        commandList.add(driveCommand);
        commandList.add(notifyCommand);

        subsystems.add(s_IntakeSubsystem);
        subsystems.add(s_ArmSubsystem);
        subsystems.add(s_Swerve);

        SequentialCommandWrapper pickupCommands = new SequentialCommandWrapper(subsystems, commandList);
        phase = -1;
        System.out.println("phase is -1");
        System.out.println("Scheduling Pickup Sequence...");
        pickupCommands.schedule();
    }
    
    public void lastPickupSequenceFinished() {
        phase = 1;
        startPos = closestPiece;//new Point (s_Swerve.getPose().getX(), s_Swerve.getPose().getY());
        System.out.println("Last Pickup Occured At: " + startPos);
        System.out.println("phase is 1");
    }

    public void lastScoringSequenceFinished() {
        piecesScored ++;
        startPos = nodePos;
        System.out.println("Pieces Scored: " + piecesScored);
        if (pieceNum <= piecesScored) {
            if (chargeStation) {
                phase = 2;
                System.out.println("phase is 2");
            }
            else {
                phase = -1;
                System.out.println("phase is -1");
            }
        }
        else {
            phase = 0;
            System.out.println("phase is 0");
        }
    }

    public SequentialCommandWrapper generateTravelArmSequence () {

        List <CommandBase> commands = new ArrayList<>();
        List <SubsystemBase> subsystems = new ArrayList<>();
        ArmPosition travel = ArmPosition.Travel;
        ArmPosition travelSequence = ArmPosition.TravelSequence;
        commands.add(new PositionArm(s_ArmSubsystem, List.of(travelSequence, travel)));

        subsystems.add(s_ArmSubsystem);

        return new SequentialCommandWrapper(subsystems, commands);
    }

    public void generateScoringSequence(SequentialCommandWrapper armSequence, PathPlannerTrajectory trajectory) {
        CommandBase driveCommand = s_Swerve.followTrajectoryCommand(trajectory, true).alongWith(armSequence).withTimeout(4);
        CommandBase alignCommand = new AutoAlign(s_Swerve, ((scoringNode + 1) % 3 == 0), 0.25, 0.5).withTimeout(1); //calculates isCone based on scoringNode
        CommandBase outtakeCommand = new IntakeAuto(s_IntakeSubsystem, Constants.AutoConstants.intakeOutAutoConstant).withTimeout(.5);//place cone on floor
        CommandBase notifyCommand = new InstantCommand(() -> lastScoringSequenceFinished());

        List <CommandBase> commandList = new ArrayList<>();
        List <SubsystemBase> subsystems = new ArrayList<>();

        commandList.add(driveCommand);
        //commandList.add(alignCommand);
        commandList.add(outtakeCommand);
        commandList.add(notifyCommand);

        subsystems.add(s_IntakeSubsystem);
        subsystems.add(s_ArmSubsystem);
        subsystems.add(s_Swerve);
        subsystems.add(s_LimelightSubsystem);

        SequentialCommandWrapper scoringCommands = new SequentialCommandWrapper(subsystems, commandList);
        phase = -1;
        System.out.println("phase is -1");
        System.out.println("Scheduling Scoring Command...");
        scoringCommands.schedule();
    }

    private Point closestPiece(Point start) {
        double minimumDistance = 1000000;
        Point closestPoint = null;
        for (Point p : availablePieces) {
            double distance = Math.hypot(start.x - p.x, start.y - p.y);
            if (distance < minimumDistance) {
                minimumDistance = distance;
                closestPoint = p;
            }
        }
        this.closestPiece = closestPoint;
        availablePieces.remove(closestPoint);
        System.out.println("The Closest Piece is " + closestPoint);
        return closestPoint;
    }

    private Point closestNode(Point start) {
        double minimumDistance = 1000000;
        Point closestPoint = null;
        int index = 0;
        for (Point p : availableNodes) {
            index ++;
            double distance = Math.hypot(start.x - p.x, start.y - p.y);
            if (distance < minimumDistance) {
                minimumDistance = distance;
                scoringNode = index;
                closestPoint = p;
            }
        }
        availableNodes.remove(closestPoint);
        nodePos = closestPoint;
        System.out.println("The Closest Node is " + closestPoint);
        return closestPoint;
    }

    private Point closestHighTraversal(Point start) {
        double minimumDistance = 1000000;
        Point closestTraversal = null;
        for (Point p : highTraversalPoints) {
            double distance = Math.hypot(start.x - p.x, start.y - p.y);
            if (distance < minimumDistance) {
                minimumDistance = distance;
                closestTraversal = p;
            }
        }
        System.out.println("The Closest Traversal is " + closestTraversal);
        return closestTraversal;
    }

    private Point closestLowTraversal(Point start) {
        double minimumDistance = 1000000;
        Point closestTraversal = null;
        for (Point p : lowTraversalPoints) {
            double distance = Math.hypot(start.x - p.x, start.y - p.y);
            if (distance < minimumDistance) {
                minimumDistance = distance;
                closestTraversal = p;
            }
        }
        System.out.println("The Closest Traversal is " + closestTraversal);
        return closestTraversal;
    }

    private Point closestChargeClimb(Point start) {
        double minimumDistance = 1000000;
        Point closestPoint = null;
        for (Point p : chargeClimbPoints) {
            double distance = Math.hypot(start.x - p.x, start.y - p.y);
            if (distance < minimumDistance) {
                minimumDistance = distance;
                closestPoint = p;
            }
        }
        return closestPoint;
    }

    public void generateBalanceSequence(SequentialCommandWrapper armSequence, PathPlannerTrajectory trajectory) {
        CommandBase driveCommand = s_Swerve.followTrajectoryCommand(trajectory, true).alongWith(armSequence).withTimeout(4);
        CommandBase autoSetup = new AutoBalSetupInvert(s_Swerve, true, true).withTimeout(1);
        CommandBase autoBalance = new AutoBalance(s_Swerve, true, true);

        List <CommandBase> commandList = new ArrayList<>();
        List <SubsystemBase> subsystems = new ArrayList<>();

        commandList.add(driveCommand);
        commandList.add(autoSetup);
        commandList.add(autoBalance);

        subsystems.add(s_ArmSubsystem);
        subsystems.add(s_Swerve);

        SequentialCommandWrapper autoBalanceCommands = new SequentialCommandWrapper(subsystems, commandList);
        phase = -1;
        autoBalanceCommands.schedule();
    }

    private boolean intersectsChargeStation(Point startPos, Point endPos){
        double highY = Constants.FieldConstants.lowXHighYTraversal.y;
        double lowX = Constants.FieldConstants.lowXHighYTraversal.x;
        double lowY = Constants.FieldConstants.highXLowYTraversal.y;
        double highX = Constants.FieldConstants.highXLowYTraversal.x;
        Point start;
        Point end;
        if (startPos.x > endPos.x) {
            start = endPos;
            end = startPos;
        }
        else {
            start = startPos;
            end = endPos;
        }
        if (start.x < lowX) {
            if (start.y < highY && start.y > lowY) {
                return true;
            }
        }
        else if (start.x < highX) {
            if (end.y < highY && end.y > lowY) {
                return true;
            }
        }
        return false;
    }

    private PathPlannerTrajectory generatePath(List <Point> coordinates, double startingRot, double endingRot) {
        List <PathPoint> pathPoints = new ArrayList<PathPoint>();
        System.out.println("Generating Path...");
        for (Point p : coordinates) {
            System.out.println("Each Point " + p);
            pathPoints.add(new PathPoint(new Translation2d(p.x, p.y), Rotation2d.fromDegrees(endingRot)));
        }
        return PathPlanner.generatePath(
        new PathConstraints(1, 1), 
        pathPoints
        );
    }
    
    @Override
    public void execute(){
        List <Point> trajectoryPoints = new ArrayList<Point>();
        Point endPos = null;
        switch (phase) {
            //scoring to pickup
            case 0:
                trajectoryPoints.add(startPos);
                endPos = closestPiece(startPos);
                if (intersectsChargeStation(startPos, endPos)) {
                    trajectoryPoints.add(closestLowTraversal(startPos));
                    if (intersectsChargeStation(trajectoryPoints.get(1), endPos)) {
                        trajectoryPoints.add(closestHighTraversal(trajectoryPoints.get(1)));
                    }
                }
                trajectoryPoints.add(endPos);
                Point last = trajectoryPoints.get(trajectoryPoints.size() - 1);
                Point secondToLast = trajectoryPoints.get(trajectoryPoints.size() - 2);
        
                double angle = Math.atan2(last.x - secondToLast.x, last.y - secondToLast.y);

                PathPlannerTrajectory trajectory = generatePath(trajectoryPoints, angle, angle);
                SequentialCommandWrapper armSequence = generatePickupArmSequence();
                generatePickupSequence(armSequence, trajectory);
                break;

            //pickup to scoring
            case 1:
                //startPos = new Point(s_Swerve.getPose().getX(), s_Swerve.getPose().getY());
                trajectoryPoints.add(startPos);
                endPos = closestNode(startPos);
                if (intersectsChargeStation(startPos, endPos)) {
                    trajectoryPoints.add(closestHighTraversal(startPos));
                    if (intersectsChargeStation(trajectoryPoints.get(1), endPos)) {
                        trajectoryPoints.add(closestLowTraversal(trajectoryPoints.get(1)));
                    }
                }
                trajectoryPoints.add(endPos);
        
                PathPlannerTrajectory trajectoryScoring = generatePath(trajectoryPoints, 0, 0);
                SequentialCommandWrapper armSequenceScoring = generateTravelArmSequence();
                generateScoringSequence(armSequenceScoring, trajectoryScoring);
                break;

            //scoring to charge station
            case 2:
                trajectoryPoints.add(startPos);
                endPos = closestChargeClimb(startPos);
                trajectoryPoints.add(endPos);

                PathPlannerTrajectory trajectoryClimb = generatePath(trajectoryPoints, 180, 180);
                SequentialCommandWrapper armSequenceClimb = generateTravelArmSequence();
                generateBalanceSequence(armSequenceClimb, trajectoryClimb);
                break;
        
            default:
                break;
        }
    }

    public class SequentialCommandWrapper extends SequentialCommandGroup {
        public SequentialCommandWrapper(List <SubsystemBase> subsystems, List <CommandBase> commands) {
            for (SubsystemBase subsystem:subsystems) {
                addRequirements(subsystem);
            }
            for (CommandBase command:commands){
                addCommands(command);
            }
            
          }
    }

}
