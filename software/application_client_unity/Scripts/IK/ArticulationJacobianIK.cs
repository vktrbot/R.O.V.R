using System;
using UnityEngine;

public class ArticulationJacobianIK : MonoBehaviour {
    public enum DriveAxis {
        X,
        Y,
        Z
    }

    [Serializable]
    public class JointConfig
    {
        public ArticulationBody body;
        public DriveAxis driveAxis = DriveAxis.X;
        public bool invert;
        public bool excludeFromPositionSolve;

        [Header("Soft Limits (deg)")]
        public bool useSoftLimits;
        public float softLowerLimit = -180f;
        public float softUpperLimit = 180f;

        [Header("Speed Limit")]
        public bool useCustomMaxDegreesPerSecond;
        [Min(0f)] public float maxDegreesPerSecond = 25f;
    }

    [Header("Chain")]
    [SerializeField] private JointConfig[] joints;
    [SerializeField] private Transform endEffector;
    [SerializeField] private Transform target;

    [Header("Solver")]
    [SerializeField, Min(1f)] private float solverRateHz = 15f;
    [SerializeField] private float positionTolerance = 0.005f;
    [SerializeField] private float stepScale = 1.0f;
    [SerializeField] private float damping = 0.1f;
    [SerializeField, Min(0f)] private float defaultMaxDegreesPerSecond = 25f;
    [SerializeField] private bool solveInFixedUpdate = true;
    [SerializeField] private bool clampTargetToEstimatedReach = true;

    [Header("Command Handshake")]
    [SerializeField] private bool waitForJointSettle = true;
    [SerializeField, Min(0f)] private float jointDeadbandDegrees = 0.5f;
    [SerializeField, Min(0f)] private float reachedJointToleranceDegrees = 1f;
    [SerializeField, Min(0f)] private float settleTimeoutSeconds = 0.75f;

    [Header("Step Safety")]
    [SerializeField] private bool checkErrorReduction = true;
    [SerializeField, Min(0f)] private float requiredErrorImprovement = 0.0005f;
    [SerializeField, Range(0.1f, 1f)] private float stepScaleOnNoImprovement = 0.5f;
    [SerializeField, Range(0.05f, 1f)] private float minAdaptiveStepScale = 0.1f;
    [SerializeField, Range(0f, 1f)] private float adaptiveRecoveryPerStep = 0.1f;

    [Header("Debug")]
    [SerializeField] private bool drawAxes = true;
    [SerializeField] private bool verboseLogging;

    private bool _hasLoggedSetupIssues;
    private float _solveAccumulator;
    private bool _waitingForPreviousCommand;
    private float _settleTimer;
    private float _pendingCommandErrorMagnitude = float.PositiveInfinity;
    private float _adaptiveStepScale = 1f;
    private bool[] _lastCommandedJoints = Array.Empty<bool>();

    private void FixedUpdate() {
        if (solveInFixedUpdate)
            TickSolver(Time.fixedDeltaTime);
    }

    private void Update() {
        if (!solveInFixedUpdate)
            TickSolver(Time.deltaTime);
    }

    [ContextMenu("Solve IK Once")]
    public void SolveIK() {
        float dt = solverRateHz > Mathf.Epsilon ? 1f / solverRateHz : 0.02f;
        SolveIKInternal(dt);
    }

    private void TickSolver(float dt) {
        if (dt <= 0f)
            return;

        if (solverRateHz <= Mathf.Epsilon) {
            SolveIKInternal(dt);
            return;
        }

        float interval = 1f / solverRateHz;
        _solveAccumulator += dt;

        if (_solveAccumulator < interval)
            return;

        float solveDt = _solveAccumulator;
        _solveAccumulator = 0f;
        SolveIKInternal(solveDt);
    }

    private void SolveIKInternal(float solverDt) {
        if (!ValidateSetup())
            return;

        EnsureCommandMaskSize();

        Vector3 currentPos = endEffector.position;
        float reach = EstimateReach();
        Vector3 desiredTarget = GetEffectiveTargetPosition(reach);
        Vector3 error = desiredTarget - currentPos;

        if (waitForJointSettle && _waitingForPreviousCommand) {
            if (AreLastCommandedJointsNearTarget()) {
                _waitingForPreviousCommand = false;
                _settleTimer = 0f;
                EvaluateErrorReduction(error.magnitude);
            } else {
                _settleTimer += solverDt;

                if (settleTimeoutSeconds > 0f && _settleTimer >= settleTimeoutSeconds) {
                    _waitingForPreviousCommand = false;
                    _settleTimer = 0f;
                    EvaluateErrorReduction(error.magnitude);

                    if (verboseLogging) {
                        Debug.LogWarning($"[{nameof(ArticulationJacobianIK)}] Settle timeout hit ({settleTimeoutSeconds:F2}s). Continuing with next IK step.", this);
                    }
                } else {
                    return;
                }
            }
        }

        if (error.magnitude <= positionTolerance)
            return;

        // Articulation drives do not update the hierarchy pose immediately after
        // setting drive targets, so multiple IK iterations in the same frame reuse
        // the same stale pose and tend to overshoot badly.
        float[] deltaRad = SolveLimitAwareStep(error, solverDt);
        bool issuedCommand = ApplyDeltaAngles(deltaRad, solverDt);

        if (issuedCommand && waitForJointSettle) {
            _waitingForPreviousCommand = true;
            _settleTimer = 0f;
            _pendingCommandErrorMagnitude = error.magnitude;
        }
    }

    private bool ValidateSetup() {
        bool foundIssue = false;

        if (endEffector == null || target == null || joints == null || joints.Length == 0) {
            LogSetupIssue("IK setup is incomplete. Assign joints, end effector, and target.");
            return false;
        }

        for (int i = 0; i < joints.Length; i++) {
            if (joints[i] == null || joints[i].body == null) {
                LogSetupIssue($"Joint {i} is missing an ArticulationBody reference.");
                return false;
            }

            if (!TryGetEffectiveLimits(joints[i], out float effectiveLower, out float effectiveUpper)) {
                foundIssue = true;
                LogSetupIssue($"Joint {i} ({joints[i].body.name}) has invalid effective limits after soft-limit merge.");
            }

            ClampDriveTargetToLimits(joints[i]);

            if (IsExcludedFromPositionSolve(joints[i]))
                continue;

            if (!HasUsableDrive(joints[i])) {
                foundIssue = true;
                LogSetupIssue($"Joint {i} ({joints[i].body.name}) is not drivable on {joints[i].driveAxis}. Effective limits [{effectiveLower:F1}, {effectiveUpper:F1}] plus stiffness/damping/forceLimit must allow motion.");
            }
        }

        float reach = EstimateReach();
        float distanceToTarget = Vector3.Distance(GetJointWorldPosition(joints[0]), target.position);
        if (reach > 0f && distanceToTarget > reach * 1.25f) {
            foundIssue = true;
            LogSetupIssue($"Target may be unreachable. Distance from base is {distanceToTarget:F3}m while estimated reach is {reach:F3}m.");
        }

        if (!foundIssue)
            _hasLoggedSetupIssues = false;

        return true;
    }

    private void EvaluateErrorReduction(float currentErrorMagnitude) {
        if (!checkErrorReduction)
            return;

        if (float.IsInfinity(_pendingCommandErrorMagnitude))
            return;

        float improvedBy = _pendingCommandErrorMagnitude - currentErrorMagnitude;
        bool improvedEnough = improvedBy >= requiredErrorImprovement;

        if (improvedEnough) {
            _adaptiveStepScale = Mathf.MoveTowards(_adaptiveStepScale, 1f, adaptiveRecoveryPerStep);
        } else {
            _adaptiveStepScale = Mathf.Max(minAdaptiveStepScale, _adaptiveStepScale * stepScaleOnNoImprovement);

            if (verboseLogging) {
                Debug.LogWarning($"[{nameof(ArticulationJacobianIK)}] Cartesian error did not improve enough ({improvedBy:F4}m). Reducing adaptive step to {_adaptiveStepScale:F2}.", this);
            }
        }

        _pendingCommandErrorMagnitude = float.PositiveInfinity;
    }

    /// <summary>
    /// Solves a single DLS step for position-only IK.
    /// J is 3 x N
    /// deltaTheta = J^T * (J*J^T + lambda^2 * I)^-1 * error
    /// </summary>
    private float[] SolveLimitAwareStep(Vector3 positionError, float solverDt) {
        bool[] blocked = new bool[joints.Length];
        float[] delta = Array.Empty<float>();

        // Re-solve a couple of times if the first pass tries to push already
        // saturated joints farther into their limits.
        for (int pass = 0; pass < joints.Length; pass++) {
            delta = SolveDampedLeastSquaresStep(positionError, blocked);
            if (!BlockSaturatedJoints(delta, blocked, solverDt))
                break;
        }

        return delta;
    }

    private float[] SolveDampedLeastSquaresStep(Vector3 positionError, bool[] blocked) {
        int n = joints.Length;

        // Jacobian: 3 x N
        float[,] J = new float[3, n];

        Vector3 eePos = endEffector.position;

        for (int i = 0; i < n; i++) {
            JointConfig joint = joints[i];
            if ((blocked != null && blocked[i]) || IsExcludedFromPositionSolve(joint) || !HasUsableDrive(joint))
                continue;

            Vector3 jointPos = GetJointWorldPosition(joint);
            Vector3 axisWorld = GetJointAxisWorld(joint).normalized;
            float sign = joint.invert ? -1f : 1f;

            // Jv = axis x (p_ee - p_joint)
            Vector3 column = Vector3.Cross(axisWorld, eePos - jointPos) * sign;

            J[0, i] = column.x;
            J[1, i] = column.y;
            J[2, i] = column.z;
        }

        // A = J * J^T + lambda^2 * I, where A is 3x3
        float[,] A = new float[3, 3];
        for (int r = 0; r < 3; r++) {
            for (int c = 0; c < 3; c++)
            {
                float sum = 0f;
                for (int k = 0; k < n; k++)
                    sum += J[r, k] * J[c, k];

                A[r, c] = sum;
            }
        }

        float lambda2 = damping * damping;
        A[0, 0] += lambda2;
        A[1, 1] += lambda2;
        A[2, 2] += lambda2;

        if (!Invert3x3(A, out float[,] Ainv))
            return new float[n];

        // y = A^-1 * error
        float[] e = { positionError.x, positionError.y, positionError.z };
        float[] y = Multiply3x3Vector(Ainv, e);

        // deltaTheta = J^T * y
        float[] delta = new float[n];
        for (int i = 0; i < n; i++) {
            float v =
                J[0, i] * y[0] +
                J[1, i] * y[1] +
                J[2, i] * y[2];

            delta[i] = v * stepScale * _adaptiveStepScale;
        }

        return delta;
    }

    private bool BlockSaturatedJoints(float[] deltaRad, bool[] blocked, float solverDt) {
        bool anyNewlyBlocked = false;

        for (int i = 0; i < joints.Length; i++) {
            if (blocked[i])
                continue;

            JointConfig joint = joints[i];
            if (IsExcludedFromPositionSolve(joint) || !HasUsableDrive(joint))
                continue;

            float sign = joint.invert ? -1f : 1f;
            float deltaDeg = deltaRad[i] * Mathf.Rad2Deg * sign;
            float maxDeltaDeg = GetMaxDeltaDegreesForStep(joint, solverDt);
            deltaDeg = Mathf.Clamp(deltaDeg, -maxDeltaDeg, maxDeltaDeg);

            if (Mathf.Approximately(deltaDeg, 0f))
                continue;

            ArticulationDrive drive = GetDrive(joint);
            if (!TryGetEffectiveLimits(joint, out float lowerLimit, out float upperLimit))
                continue;

            float clampedTarget = Mathf.Clamp(drive.target + deltaDeg, lowerLimit, upperLimit);

            // If the joint is already on a limit and this step still wants to
            // push farther into that same limit, remove it from the solve so the
            // remaining joints can take over.
            if (Mathf.Abs(clampedTarget - drive.target) <= 0.001f) {
                blocked[i] = true;
                anyNewlyBlocked = true;
            }
        }

        return anyNewlyBlocked;
    }

    private bool ApplyDeltaAngles(float[] deltaRad, float solverDt) {
        Array.Clear(_lastCommandedJoints, 0, _lastCommandedJoints.Length);
        bool anyCommandIssued = false;

        for (int i = 0; i < joints.Length; i++) {
            JointConfig joint = joints[i];
            if (IsExcludedFromPositionSolve(joint) || !HasUsableDrive(joint))
                continue;

            float sign = joint.invert ? -1f : 1f;
            float deltaDeg = deltaRad[i] * Mathf.Rad2Deg * sign;
            float maxDeltaDeg = GetMaxDeltaDegreesForStep(joint, solverDt);
            deltaDeg = Mathf.Clamp(deltaDeg, -maxDeltaDeg, maxDeltaDeg);

            if (Mathf.Abs(deltaDeg) < jointDeadbandDegrees)
                continue;

            ArticulationDrive drive = GetDrive(joint);
            if (!TryGetEffectiveLimits(joint, out float lowerLimit, out float upperLimit))
                continue;

            float unclampedTarget = drive.target + deltaDeg;
            float clampedTarget = Mathf.Clamp(unclampedTarget, lowerLimit, upperLimit);

            if (Mathf.Abs(clampedTarget - drive.target) < jointDeadbandDegrees)
                continue;

            drive.target = clampedTarget;
            SetDrive(joint, drive);

            anyCommandIssued = true;
            _lastCommandedJoints[i] = true;
            LogJointClamp(joint, unclampedTarget, clampedTarget, lowerLimit, upperLimit);
        }

        return anyCommandIssued;
    }

    private Vector3 GetJointAxisWorld(JointConfig joint) {
        Vector3 axisLocal = joint.driveAxis switch {
            DriveAxis.X => Vector3.right,
            DriveAxis.Y => Vector3.up,
            DriveAxis.Z => Vector3.forward,
            _ => Vector3.right
        };

        Quaternion anchorRotationWorld = joint.body.transform.rotation * joint.body.anchorRotation;
        return anchorRotationWorld * axisLocal;
    }

    private static Vector3 GetJointWorldPosition(JointConfig joint) {
        return joint.body.transform.TransformPoint(joint.body.anchorPosition);
    }

    private bool HasUsableDrive(JointConfig joint) {
        ArticulationDrive drive = GetDrive(joint);
        if (!TryGetEffectiveLimits(joint, out float lowerLimit, out float upperLimit))
            return false;

        if (Mathf.Approximately(lowerLimit, upperLimit))
            return false;

        if (drive.forceLimit <= 0f)
            return false;

        return drive.stiffness > 0f || drive.damping > 0f;
    }

    private static bool IsExcludedFromPositionSolve(JointConfig joint) {
        return joint != null && joint.excludeFromPositionSolve;
    }

    private static ArticulationDrive GetDrive(JointConfig joint) {
        return joint.driveAxis switch {
            DriveAxis.X => joint.body.xDrive,
            DriveAxis.Y => joint.body.yDrive,
            DriveAxis.Z => joint.body.zDrive,
            _ => joint.body.xDrive
        };
    }

    private static void SetDrive(JointConfig joint, ArticulationDrive drive) {
        switch (joint.driveAxis) {
            case DriveAxis.X:
                joint.body.xDrive = drive;
                break;
            case DriveAxis.Y:
                joint.body.yDrive = drive;
                break;
            case DriveAxis.Z:
                joint.body.zDrive = drive;
                break;
        }
    }

    private void ClampDriveTargetToLimits(JointConfig joint) {
        ArticulationDrive drive = GetDrive(joint);
        if (!TryGetEffectiveLimits(joint, out float lowerLimit, out float upperLimit))
            return;
        drive.target = Mathf.Clamp(drive.target, lowerLimit, upperLimit);
        SetDrive(joint, drive);
    }

    private bool TryGetEffectiveLimits(JointConfig joint, out float lowerLimit, out float upperLimit) {
        ArticulationDrive drive = GetDrive(joint);
        lowerLimit = drive.lowerLimit;
        upperLimit = drive.upperLimit;

        if (joint.useSoftLimits) {
            lowerLimit = Mathf.Max(lowerLimit, joint.softLowerLimit);
            upperLimit = Mathf.Min(upperLimit, joint.softUpperLimit);
        }

        return lowerLimit <= upperLimit;
    }

    private float GetMaxDeltaDegreesForStep(JointConfig joint, float solverDt) {
        float maxDegPerSecond = joint.useCustomMaxDegreesPerSecond
            ? joint.maxDegreesPerSecond
            : defaultMaxDegreesPerSecond;

        maxDegPerSecond = Mathf.Max(0f, maxDegPerSecond);
        return maxDegPerSecond * Mathf.Max(0f, solverDt);
    }

    private void EnsureCommandMaskSize() {
        int desiredLength = joints != null ? joints.Length : 0;
        if (_lastCommandedJoints.Length == desiredLength)
            return;

        _lastCommandedJoints = new bool[desiredLength];
    }

    private bool AreLastCommandedJointsNearTarget() {
        if (joints == null || joints.Length == 0 || _lastCommandedJoints.Length != joints.Length)
            return true;

        for (int i = 0; i < joints.Length; i++) {
            if (!_lastCommandedJoints[i])
                continue;

            JointConfig joint = joints[i];
            if (!HasUsableDrive(joint))
                continue;

            if (!TryGetJointPositionDegrees(joint, out float measuredDeg))
                continue;
            float targetDeg = GetDrive(joint).target;
            float errorDeg = Mathf.Abs(Mathf.DeltaAngle(measuredDeg, targetDeg));

            if (errorDeg > reachedJointToleranceDegrees)
                return false;
        }

        // If we could not read any measurable joint angle, do not block forever.
        return true;
    }

    private static bool TryGetJointPositionDegrees(JointConfig joint, out float angleDegrees) {
        angleDegrees = 0f;

        if (joint == null || joint.body == null)
            return false;

        int dofCount = joint.body.dofCount;
        if (dofCount <= 0)
            return false;

        int index;
        if (dofCount == 1) {
            index = 0;
        } else {
            index = joint.driveAxis switch {
                DriveAxis.X => 0,
                DriveAxis.Y => 1,
                DriveAxis.Z => 2,
                _ => 0
            };
        }

        if (index < 0 || index >= dofCount)
            return false;

        ArticulationReducedSpace jointPos = joint.body.jointPosition;
        angleDegrees = jointPos[index] * Mathf.Rad2Deg;
        return true;
    }

    private Vector3 GetEffectiveTargetPosition(float reach) {
        if (!clampTargetToEstimatedReach || reach <= 0f || joints == null || joints.Length == 0)
            return target.position;

        Vector3 basePosition = GetJointWorldPosition(joints[0]);
        Vector3 toTarget = target.position - basePosition;
        float distance = toTarget.magnitude;
        float maxReach = reach * 0.98f;

        if (distance <= maxReach || distance <= Mathf.Epsilon)
            return target.position;

        return basePosition + toTarget / distance * maxReach;
    }

    private float EstimateReach() {
        float reach = 0f;
        Vector3 previous = GetJointWorldPosition(joints[0]);

        for (int i = 1; i < joints.Length; i++) {
            Vector3 current = GetJointWorldPosition(joints[i]);
            reach += Vector3.Distance(previous, current);
            previous = current;
        }

        reach += Vector3.Distance(previous, endEffector.position);
        return reach;
    }

    private void LogSetupIssue(string message) {
        if (_hasLoggedSetupIssues)
            return;

        _hasLoggedSetupIssues = true;
        Debug.LogWarning($"[{nameof(ArticulationJacobianIK)}] {message}", this);
    }

    private void LogJointClamp(JointConfig joint, float unclampedTarget, float clampedTarget, float lowerLimit, float upperLimit) {
        if (!verboseLogging)
            return;

        if (!Mathf.Approximately(unclampedTarget, clampedTarget)) {
            Debug.Log($"[{nameof(ArticulationJacobianIK)}] {joint.body.name} clamped on {joint.driveAxis}: {unclampedTarget:F2} -> {clampedTarget:F2} within [{lowerLimit:F2}, {upperLimit:F2}].", joint.body);
        }
    }

    private static float[] Multiply3x3Vector(float[,] m, float[] v) {
        return new float[] {
            m[0,0] * v[0] + m[0,1] * v[1] + m[0,2] * v[2],
            m[1,0] * v[0] + m[1,1] * v[1] + m[1,2] * v[2],
            m[2,0] * v[0] + m[2,1] * v[1] + m[2,2] * v[2]
        };
    }

    private static bool Invert3x3(float[,] m, out float[,] inv) {
        inv = new float[3, 3];

        float a = m[0, 0], b = m[0, 1], c = m[0, 2];
        float d = m[1, 0], e = m[1, 1], f = m[1, 2];
        float g = m[2, 0], h = m[2, 1], i = m[2, 2];

        float A = (e * i - f * h);
        float B = -(d * i - f * g);
        float C = (d * h - e * g);
        float D = -(b * i - c * h);
        float E = (a * i - c * g);
        float F = -(a * h - b * g);
        float G = (b * f - c * e);
        float H = -(a * f - c * d);
        float I = (a * e - b * d);

        float det = a * A + b * B + c * C;

        if (Mathf.Abs(det) < 1e-8f)
            return false;

        float invDet = 1f / det;

        inv[0, 0] = A * invDet;
        inv[0, 1] = D * invDet;
        inv[0, 2] = G * invDet;

        inv[1, 0] = B * invDet;
        inv[1, 1] = E * invDet;
        inv[1, 2] = H * invDet;

        inv[2, 0] = C * invDet;
        inv[2, 1] = F * invDet;
        inv[2, 2] = I * invDet;

        return true;
    }

    private void OnDrawGizmos() {
        if (!drawAxes || joints == null)
            return;

        foreach (var joint in joints) {
            if (joint == null || joint.body == null)
                continue;

            Vector3 pos = GetJointWorldPosition(joint);
            Vector3 axis = GetJointAxisWorld(joint).normalized;

            Gizmos.color = Color.cyan;
            Gizmos.DrawLine(pos, pos + axis * 0.12f);
            Gizmos.DrawSphere(pos, 0.01f);
        }

        if (endEffector != null && target != null) {
            Gizmos.color = Color.yellow;
            Gizmos.DrawLine(endEffector.position, target.position);
        }
    }
}
