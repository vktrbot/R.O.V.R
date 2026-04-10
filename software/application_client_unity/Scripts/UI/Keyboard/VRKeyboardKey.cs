using System;
using TMPro;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.UI;

namespace UI {
    public class VRKeyboardKey : MonoBehaviour, IPointerDownHandler {
        public enum KeyType {
            Character,
            Space,
            Backspace,
            Shift,
            Enter,
            Close
        }

        [SerializeField] private KeyType keyType = KeyType.Character;
        [SerializeField] private string character = "a";
        [SerializeField] private string customCharacterOnRegister = "";
        [SerializeField] private TMP_Text label;

        public void Press() {
            if (VRKeyboard.Instance == null)
                return;

            switch (keyType) {
                case KeyType.Character:
                    VRKeyboard.Instance.InsertCharacter(GetCharacterByRegister(VRKeyboard.Instance.ShiftEnabled));
                    break;
                case KeyType.Space:
                    VRKeyboard.Instance.InsertSpace();
                    break;
                case KeyType.Backspace:
                    VRKeyboard.Instance.Backspace();
                    break;
                case KeyType.Shift:
                    VRKeyboard.Instance.ToggleShift();
                    break;
                case KeyType.Enter:
                    VRKeyboard.Instance.Submit();
                    break;
                case KeyType.Close:
                    VRKeyboard.Instance.Close();
                    break;
            }
        }

        public void RefreshLabel(bool shiftEnabled) {
            if (label == null)
                return;
            label.text = GetCharacterByRegister(shiftEnabled);
        }

        public string GetCharacterByRegister(bool register) {
            if (keyType == KeyType.Character) {
                if (customCharacterOnRegister != "" && register) {
                    return customCharacterOnRegister;
                }
                return register ? character.ToUpper() : character.ToLower();
            }

            return character;
        }

        public void OnPointerDown(PointerEventData eventData) {
            Press();
        }
    }
}
