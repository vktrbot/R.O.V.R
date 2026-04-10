using TMPro;
using UnityEngine;
using UnityEngine.EventSystems;

namespace UI {
    

    [RequireComponent(typeof(TMP_InputField))]
    public class VRKeyboardInputTarget : MonoBehaviour, IPointerClickHandler, ISelectHandler
    {
        private TMP_InputField _inputField;

        private void Awake() {
            _inputField = GetComponent<TMP_InputField>();
        }

        public void OnPointerClick(PointerEventData eventData) {
            OpenKeyboard();
        }

        public void OnSelect(BaseEventData eventData) {
            OpenKeyboard();
        }

        private void OpenKeyboard() {
            if (VRKeyboard.Instance == null) {
                Debug.LogWarning("VRKeyboard instance not found in scene.");
                return;
            }

            VRKeyboard.Instance.Open(_inputField);
        }
    }
}