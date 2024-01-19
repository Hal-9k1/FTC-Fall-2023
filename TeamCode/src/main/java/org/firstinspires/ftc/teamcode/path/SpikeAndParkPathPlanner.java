package org.firstinspires.ftc.teamcode.path;

import org.firstinspires.ftc.teamcode.MatrixMagic;
import org.firstinspires.ftc.teamcode.navigator.RobotNavigator;

import java.util.ArrayDeque;
import java.util.List;

import javax.vecmath.Matrix3d;
import javax.vecmath.Matrix4d;
import javax.vecmath.Vector3d;

public class SpikeAndParkPathPlanner implements PathPlanner {
    private static final double SPIKE_LOCATION_TOLERANCE = 0.1; // meters
    private static final Matrix4d[] SPIKE_TRANSFORMS_START_SPACE; // 0 left 1 middle 2 right
    // on the corresponding alliance's wall, facing the center of the field:
    private static final Matrix4d RED_ALLIANCE_ORIGIN_FIELD_SPACE;
    private static final Matrix4d BLUE_ALLIANCE_ORIGIN_FIELD_SPACE;
    // defined to be the starting point of a robot in that position
    private static final Matrix4d BACK_START_ORIGIN_ALLIANCE_SPACE;
    private static final Matrix4d FRONT_START_ORIGIN_ALLIANCE_SPACE;
    static {
        Matrix3d id = new Matrix3d();
        id.setIdentity();
        SPIKE_TRANSFORMS_START_SPACE = new Matrix4d[3];
        SPIKE_TRANSFORMS_START_SPACE[0] = new Matrix4d(id, new Vector3d(0.61 * 0.5, 0.61, 0.0), 1.0);
        SPIKE_TRANSFORMS_START_SPACE[1] = new Matrix4d(id, new Vector3d(0.0, 0.61 * 1.5, 0.0), 1.0);
        SPIKE_TRANSFORMS_START_SPACE[2] = new Matrix4d(id, new Vector3d(0.61 * -0.5, 0.61, 0.0), 1.0);

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
    private final Matrix4d[] spikeTransforms;
    private int spikeTransformIdx; // idx in spikeTransforms
    private final Matrix4d allianceOrigin;
    private final Matrix4d startPositionOrigin;
    private boolean stopped;
    private final ArrayDeque<RobotGoal> goalQueue;
    private boolean waitingForSpikeTransform;
    public SpikeAndParkPathPlanner(RobotNavigator navigator, Alliance alliance,
            StartingPosition startingPosition) {
        this.navigator = navigator;
        allianceOrigin = alliance == Alliance.RED ? RED_ALLIANCE_ORIGIN_FIELD_SPACE
            : BLUE_ALLIANCE_ORIGIN_FIELD_SPACE;
        startPositionOrigin = startingPosition == StartingPosition.BACK
            ? BACK_START_ORIGIN_ALLIANCE_SPACE : FRONT_START_ORIGIN_ALLIANCE_SPACE;
        spikeTransforms = new Matrix4d[3];
        for (int i = 0; i < spikeTransforms.length; ++i) {
          spikeTransforms[i] = new Matrix4d(allianceOrigin);
          spikeTransforms[i].mul(startPositionOrigin);
          spikeTransforms[i].mul(SPIKE_TRANSFORMS_START_SPACE[i]);
        }
        spikeTransformIdx = -1;
        stopped = false;
        goalQueue = new ArrayDeque<>();
        waitingForSpikeTransform = false;
        queueInitialGoals();
        navigator.setGoal(goalQueue.poll());
    }
    @Override
    public boolean tick() {
        if (stopped) {
            return true;
        }
        if (waitingForSpikeTransform) {
            return false; // nothing to do now, but check back later
        }
        if (navigator.tick()) {
            RobotGoal nextGoal = goalQueue.poll();
            if (nextGoal == null) {
              if (spikeTransformIdx == -1) {
                  waitingForSpikeTransform = true;
                  return false;
              } else {
                stopped = true;
                return true;
              }
            }
            navigator.setGoal(nextGoal);
        }
        return false;
    }

    @Override
    public boolean acceptSpikeTransforms(List<Matrix4d> transformList) {
        if (spikeTransformIdx != -1) {
            return true; // already found it
        }
        double bestSpikeSqDist = Double.MAX_VALUE;
        int bestSpikeTransformIdx = -1;
        for (Matrix4d detectionTransform : transformList) {
            for (int i = 0; i < spikeTransforms.length; ++i) {
                Matrix4d relTransform = MatrixMagic.invMul(detectionTransform, spikeTransforms[i]);
                Vector3d offset = new Vector3d();
                relTransform.get(offset);
                double sqDist = offset.lengthSquared();
                if (sqDist < Math.pow(SPIKE_LOCATION_TOLERANCE, 2)
                        && (bestSpikeTransformIdx == -1 || sqDist < bestSpikeSqDist)) {
                    bestSpikeSqDist = sqDist;
                    bestSpikeTransformIdx = i;
                }
            }
        }
        if (bestSpikeTransformIdx != -1) {
            spikeTransformIdx = bestSpikeTransformIdx;
            waitingForSpikeTransform = false;
            queuePostSpikeGoals();
            return true; // found it
        } else {
            return false;
        }
    }

    private void queueInitialGoals() {
        Matrix3d id = new Matrix3d();
        id.setIdentity();
        pushStartSpaceGoal(new Matrix4d(id, new Vector3d(0.61, 0, 0), 1.0));
    }
    private void queuePostSpikeGoals() {
        Matrix3d id = new Matrix3d();
        id.setIdentity();
        pushStartSpaceGoal(SPIKE_TRANSFORMS_START_SPACE[spikeTransformIdx]);
        pushStartSpaceGoal(new Matrix4d(id, new Vector3d(2.5 * 0.61, 0, 0), 1.0));
        Matrix3d endASGoalRotMat = new Matrix3d();
        endASGoalRotMat.rotZ(Math.PI);
        pushAllianceSpaceGoal(new Matrix4d(endASGoalRotMat, new Vector3d(2 * 0.61, 2.5 * 0.61, 0),
              1.0));
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
