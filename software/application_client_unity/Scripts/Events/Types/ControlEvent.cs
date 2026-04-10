using UnityEngine;

namespace Events.Types {
    public class DeadManSwitch : IEvent {
        public bool IsActive;
        public DeadManSwitch(bool isActive) {
            IsActive = isActive;
        }
    }

    public class PositioningControllerEvent : ControllerEvent {
        public Vector3 Position;
        public Quaternion Rotation;

        public PositioningControllerEvent(Side side, Vector3 position, Quaternion rotation) : base(side) {
            Position = position;
            Rotation = rotation;
        }
    }
    
    // CONTROLLER EVENTS
    
    public enum Side { RIGHT, LEFT }

    public abstract class ControllerEvent : IEvent {
        public Side Side;

        public ControllerEvent(Side side) {
            Side = side;
        }
    }

    public class OnGrabEvent : ControllerEvent {
        public bool IsActive;
        
        public OnGrabEvent(Side side, bool isActive) : base(side) {
            IsActive = isActive;
        }
    }

    public class OnPrimaryButtonEvent : ControllerEvent {
        public bool IsActive;

        public OnPrimaryButtonEvent(Side side, bool isActive) : base(side) {
            IsActive = isActive;
        }
    }

    public class OnSecondaryButtonEvent : ControllerEvent {
        public bool IsActive;

        public OnSecondaryButtonEvent(Side side, bool isActive) : base(side) {
            IsActive = isActive;
        }
    }

    public class OnStickEvent : ControllerEvent {
        public Vector2 Position;

        public OnStickEvent(Side side, Vector2 position) : base(side) {
            Position = position;
        }
    }
}