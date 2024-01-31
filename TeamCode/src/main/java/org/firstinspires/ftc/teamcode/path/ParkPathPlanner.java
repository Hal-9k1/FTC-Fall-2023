package org.firstinspires.ftc.teamcode.path;

import org.firstinspires.ftc.teamcode.navigator.RobotNavigator;

import java.util.ArrayDeque;
import java.util.List;

import javax.vecmath.Matrix3d;
import javax.vecmath.Matrix4d;
import javax.vecmath.Vector3d;

public class ParkPathPlanner implements PathPlanner {
    private static final Matrix4d RED_ALLIANCE_ORIGIN_FIELD_SPACE;
    private static final Matrix4d BLUE_ALLIANCE_ORIGIN_FIELD_SPACE;
    // defined to be the starting point of a robot in that position
    private static final Matrix4d BACK_START_ORIGIN_ALLIANCE_SPACE;
    private static final Matrix4d FRONT_START_ORIGIN_ALLIANCE_SPACE;
    static {
        Matrix3d id = new Matrix3d();
        id.setIdentity();

        Matrix3d redOriginRot = new Matrix3d();
        redOriginRot.rotZ(-Math.PI / 2);
        RED_ALLIANCE_ORIGIN_FIELD_SPACE = new Matrix4d(redOriginRot, new Vector3d(0, 3 * 0.61, 0), 1.0);
        RED_ALLIANCE_ORIGIN_FIELD_SPACE.mul(new Matrix4d( // flip y for red origin so +y is always backstage
          1.0,  0.0, 0.0, 0.0,
          0.0, -1.0, 0.0, 0.0,
          0.0,  0.0, 1.0, 0.0,
          0.0,  0.0, 0.0, 1.0
        ));

        Matrix3d blueOriginRot = new Matrix3d();
        blueOriginRot.rotZ(Math.PI / 2);
        BLUE_ALLIANCE_ORIGIN_FIELD_SPACE = new Matrix4d(blueOriginRot, new Vector3d(0, -3 * 0.61, 0), 1.0);

        BACK_START_ORIGIN_ALLIANCE_SPACE = new Matrix4d(id, new Vector3d(0, 1.5 * 0.61, 0), 1.0);
        FRONT_START_ORIGIN_ALLIANCE_SPACE = new Matrix4d(id, new Vector3d(0, -1.5 * 0.61, 0), 1.0);
    }
    private final RobotNavigator navigator;
    private final Matrix4d allianceOrigin;
    private final Matrix4d startPositionOrigin;
    private boolean stopped;
    private final ArrayDeque<RobotGoal> goalQueue;
    public ParkPathPlanner(RobotNavigator navigator, Alliance alliance,
            StartingPosition startingPosition) {
        this.navigator = navigator;
        allianceOrigin = alliance == Alliance.RED ? RED_ALLIANCE_ORIGIN_FIELD_SPACE
            : BLUE_ALLIANCE_ORIGIN_FIELD_SPACE;
        startPositionOrigin = startingPosition == StartingPosition.BACK
            ? BACK_START_ORIGIN_ALLIANCE_SPACE : FRONT_START_ORIGIN_ALLIANCE_SPACE;
        stopped = false;
        goalQueue = new ArrayDeque<>();
        queueInitialGoals();
        navigator.setGoal(goalQueue.poll()); // assumes at least one goal in the queue
    }
    @Override
    public boolean tick() {
        if (stopped) {
            return true;
        }
        if (navigator.tick()) {
            RobotGoal nextGoal = goalQueue.poll();
            if (nextGoal == null) {
                stopped = true;
                return true;
            }
            navigator.setGoal(nextGoal);
        }
        return false;
    }

    @Override
    public boolean acceptSpikeTransforms(List<Matrix4d> transformList) {
        return true; // don't send us these, we're not looking for them
    }

    private void queueInitialGoals() {
        Matrix3d id = new Matrix3d();
        id.setIdentity();
        pushStartSpaceGoal(new Matrix4d(id, new Vector3d(0.61 * 2.5, 0, 0), 1.0));
        Matrix3d turned = new Matrix3d();
        turned.rotZ(Math.PI / 2);
        pushStartSpaceGoal(new Matrix4d(turned, new Vector3d(0.61 * 2.5, 0, 0), 1.0));
        pushAllianceSpaceGoal(new Matrix4d(turned, new Vector3d(0.61 * 2.5, 0.61 * 2.5, 0), 1.0));
    }
    private void pushAllianceSpaceGoal(Matrix4d allianceSpaceGoalTransform) {
        Matrix4d fieldSpaceGoalTransform = new Matrix4d(allianceSpaceGoalTransform);
        fieldSpaceGoalTransform.mul(allianceOrigin);
        goalQueue.addLast(new RobotGoal(fieldSpaceGoalTransform));
    }
    private void pushStartSpaceGoal(Matrix4d startSpaceGoalTransform) {
        Matrix4d allianceSpaceGoalTransform = new Matrix4d(startSpaceGoalTransform);
        allianceSpaceGoalTransform.mul(startPositionOrigin);
        pushAllianceSpaceGoal(allianceSpaceGoalTransform);
    }
}
