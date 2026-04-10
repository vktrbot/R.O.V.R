using Events;
using Events.Types;
using UnityEngine;

namespace Services.Roboarm {
    public class JointController : MonoBehaviour {
        
        private Quaternion _homeRotation;
        
        private Listener<DeadManSwitch> _switch;
        
        private void Awake() {
            _homeRotation = transform.rotation;

            _switch = new Listener<DeadManSwitch>(OnSwitch);
            EventBus<DeadManSwitch>.Register(_switch, this);
        }
        
        private void OnSwitch(DeadManSwitch e) {
            if (!e.IsActive) {
                transform.rotation = _homeRotation;
            }
        }
    }
}