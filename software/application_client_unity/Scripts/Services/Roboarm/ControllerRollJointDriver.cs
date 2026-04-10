using Events;
using Events.Types;
using UnityEngine;

namespace Services.Roboarm {
    [RequireComponent(typeof(ArticulationBody))]
    public class ControllerRollJointDriver : MonoBehaviour {
        [Header("Controller Input")]
        [SerializeField] private Side side = Side.RIGHT;
        [SerializeField] private ArticulationJacobianIK.DriveAxis controllerRollAxis = ArticulationJacobianIK.DriveAxis.Z;
        [SerializeField] private bool useDeadManSwitch = true;

        [Header("Joint Output")]
        [SerializeField] private ArticulationJacobianIK.DriveAxis jointAxis = ArticulationJacobianIK.DriveAxis.X;
        [SerializeField] private bool invert;
        [SerializeField] private float degreesMultiplier = 1f;
        [SerializeField] private float offsetDegrees;
        [SerializeField] private bool resetToStartTargetOnRelease = true;

        private Listener<PositioningControllerEvent> _positioningController;
        private Listener<DeadManSwitch> _deadManSwitch;

        private ArticulationBody _body;
        private Quaternion _referenceRotation = Quaternion.identity;
        private float _startDriveTarget;
        private bool _isActive;
        private bool _hasReferenceRotation;

        private void Awake() {
            _body = GetComponent<ArticulationBody>();
            _startDriveTarget = GetDrive().target;
            _isActive = !useDeadManSwitch;

            _positioningController = new Listener<PositioningControllerEvent>(OnPositioning);
            EventBus<PositioningControllerEvent>.Register(_positioningController, this);

            _deadManSwitch = new Listener<DeadManSwitch>(OnDeadManSwitch);
            EventBus<DeadManSwitch>.Register(_deadManSwitch, this);
        }

        private void OnDeadManSwitch(DeadManSwitch e) {
            if (!useDeadManSwitch)
                return;

            if (e.IsActive) {
                _isActive = true;
                _hasReferenceRotation = false;
                _startDriveTarget = GetDrive().target;
                return;
            }

            _isActive = false;
            _hasReferenceRotation = false;

            if (resetToStartTargetOnRelease)
                ApplyDriveTarget(_startDriveTarget);
        }

        private void OnPositioning(PositioningControllerEvent e) {
            if (e.Side != side || !_isActive)
                return;

            if (!_hasReferenceRotation) {
                _referenceRotation = e.Rotation;
                _hasReferenceRotation = true;
                return;
            }

            Quaternion relativeRotation = Quaternion.Inverse(_referenceRotation) * e.Rotation;
            float rollDegrees = ExtractAxisAngle(relativeRotation, controllerRollAxis);
            float sign = invert ? -1f : 1f;
            float driveTarget = _startDriveTarget + sign * rollDegrees * degreesMultiplier + offsetDegrees;
            ApplyDriveTarget(driveTarget);
        }

        private void ApplyDriveTarget(float targetDegrees) {
            ArticulationDrive drive = GetDrive();
            targetDegrees = Mathf.Clamp(targetDegrees, drive.lowerLimit, drive.upperLimit);
            drive.target = targetDegrees;
            SetDrive(drive);
        }

        private ArticulationDrive GetDrive() {
            return jointAxis switch {
                ArticulationJacobianIK.DriveAxis.X => _body.xDrive,
                ArticulationJacobianIK.DriveAxis.Y => _body.yDrive,
                ArticulationJacobianIK.DriveAxis.Z => _body.zDrive,
                _ => _body.xDrive
            };
        }

        private void SetDrive(ArticulationDrive drive) {
            switch (jointAxis) {
                case ArticulationJacobianIK.DriveAxis.X:
                    _body.xDrive = drive;
                    break;
                case ArticulationJacobianIK.DriveAxis.Y:
                    _body.yDrive = drive;
                    break;
                case ArticulationJacobianIK.DriveAxis.Z:
                    _body.zDrive = drive;
                    break;
            }
        }

        private static float ExtractAxisAngle(Quaternion rotation, ArticulationJacobianIK.DriveAxis axis) {
            Vector3 localAxis = axis switch {
                ArticulationJacobianIK.DriveAxis.X => Vector3.right,
                ArticulationJacobianIK.DriveAxis.Y => Vector3.up,
                ArticulationJacobianIK.DriveAxis.Z => Vector3.forward,
                _ => Vector3.right
            };

            Vector3 projected = Vector3.Project(new Vector3(rotation.x, rotation.y, rotation.z), localAxis);
            Quaternion twist = new Quaternion(projected.x, projected.y, projected.z, rotation.w);
            float magnitude = Mathf.Sqrt(
                twist.x * twist.x +
                twist.y * twist.y +
                twist.z * twist.z +
                twist.w * twist.w);

            if (magnitude <= Mathf.Epsilon)
                return 0f;

            float invMagnitude = 1f / magnitude;
            twist = new Quaternion(
                twist.x * invMagnitude,
                twist.y * invMagnitude,
                twist.z * invMagnitude,
                twist.w * invMagnitude);

            twist.ToAngleAxis(out float angle, out Vector3 extractedAxis);
            angle = NormalizeAngle(angle);

            if (Vector3.Dot(extractedAxis, localAxis) < 0f)
                angle = -angle;

            return angle;
        }

        private static float NormalizeAngle(float angle) {
            return angle > 180f ? angle - 360f : angle;
        }
    }
}
