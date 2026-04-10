using TMPro;
using UI;
using UnityEngine;
using UnityEngine.EventSystems;
using System.Collections;

public class VRKeyboard : MonoBehaviour {
    public static VRKeyboard Instance { get; private set; }

    [Header("Root")]
    [SerializeField] private GameObject keyboardRoot;

    private TMP_InputField _currentInputField;
    private bool _restoreFocusSelectAll;
    private bool _shiftEnabled;
    private bool _isRestoringFocus;
    private Coroutine _pendingFocusRestore;

    public bool IsOpen => keyboardRoot != null && keyboardRoot.activeSelf;
    public bool ShiftEnabled => _shiftEnabled;

    private void Awake() {
        if (Instance != null && Instance != this) {
            Destroy(gameObject);
            return;
        }

        Instance = this;

        if (keyboardRoot != null)
            keyboardRoot.SetActive(false);

        RefreshKeysCase();
    }

    public void Open(TMP_InputField inputField) {
        if (inputField == null)
            return;

        if (_currentInputField != null)
            return;
        
        _currentInputField = inputField;
        _currentInputField.resetOnDeActivation = false;
        _restoreFocusSelectAll = _currentInputField.onFocusSelectAll;
        _currentInputField.onFocusSelectAll = false;
        _currentInputField.shouldHideMobileInput = true;
        _currentInputField.shouldHideSoftKeyboard = true;
        if (keyboardRoot != null)
            keyboardRoot.SetActive(true);

        RestoreInputFieldFocusImmediately();
    }

    public void Close() {
        if (keyboardRoot != null)
            keyboardRoot.SetActive(false);

        if (_currentInputField != null) {
            _currentInputField.DeactivateInputField();
            _currentInputField.resetOnDeActivation = true;
            _currentInputField.onFocusSelectAll = _restoreFocusSelectAll;
        }

        if (_pendingFocusRestore != null) {
            StopCoroutine(_pendingFocusRestore);
            _pendingFocusRestore = null;
        }

        _currentInputField = null;
        _shiftEnabled = false;
        RefreshKeysCase();
    }

    public void ToggleShift() {
        _shiftEnabled = !_shiftEnabled;
        RefreshKeysCase();
        ScheduleFocusRestore();
    }

    public void InsertCharacter(string value) {
        if (_currentInputField == null || string.IsNullOrEmpty(value))
            return;

        int start = Mathf.Min(_currentInputField.selectionStringAnchorPosition, _currentInputField.selectionStringFocusPosition);
        int end = Mathf.Max(_currentInputField.selectionStringAnchorPosition, _currentInputField.selectionStringFocusPosition);

        string text = _currentInputField.text ?? string.Empty;

        if (start != end) {
            text = text.Remove(start, end - start);
            ApplyTextState(text, start);
        }

        int caret = _currentInputField.stringPosition;
        text = _currentInputField.text ?? string.Empty;
        text = text.Insert(caret, value);

        int newCaret = caret + value.Length;
        ApplyTextState(text, newCaret);
        ScheduleFocusRestore();

        if (_shiftEnabled) {
            _shiftEnabled = false;
            RefreshKeysCase();
        }
    }

    public void InsertSpace() {
        InsertCharacter(" ");
    }

    public void Backspace() {
        if (_currentInputField == null)
            return;

        string text = _currentInputField.text ?? string.Empty;

        int start = Mathf.Min(_currentInputField.selectionStringAnchorPosition, _currentInputField.selectionStringFocusPosition);
        int end = Mathf.Max(_currentInputField.selectionStringAnchorPosition, _currentInputField.selectionStringFocusPosition);

        if (start != end) {
            text = text.Remove(start, end - start);
            ApplyTextState(text, start);
            ScheduleFocusRestore();
            return;
        }

        int caret = _currentInputField.stringPosition;
        if (caret <= 0 || text.Length == 0)
            return;

        text = text.Remove(caret - 1, 1);
        int newCaret = caret - 1;
        ApplyTextState(text, newCaret);
        ScheduleFocusRestore();
    }

    public void Submit() {
        if (_currentInputField == null)
            return;

        _currentInputField.onSubmit?.Invoke(_currentInputField.text);
        Close();
    }

    private void RefreshKeysCase() {
        var keys = GetComponentsInChildren<VRKeyboardKey>(true);
        foreach (var key in keys)
        {
            key.RefreshLabel(_shiftEnabled);
        }
    }

    private void ScheduleFocusRestore() {
        if (_currentInputField == null || !IsOpen)
            return;

        if (_pendingFocusRestore != null)
            StopCoroutine(_pendingFocusRestore);

        _pendingFocusRestore = StartCoroutine(RestoreFocusNextFrame());
    }

    private void ApplyTextState(string text, int caretPosition) {
        if (_currentInputField == null)
            return;

        text ??= string.Empty;
        caretPosition = Mathf.Clamp(caretPosition, 0, text.Length);

        _currentInputField.SetTextWithoutNotify(text);

        _currentInputField.stringPosition = caretPosition;
        _currentInputField.selectionStringAnchorPosition = caretPosition;
        _currentInputField.selectionStringFocusPosition = caretPosition;

        _currentInputField.caretPosition = caretPosition;
        _currentInputField.selectionAnchorPosition = caretPosition;
        _currentInputField.selectionFocusPosition = caretPosition;

        _currentInputField.ForceLabelUpdate();
        if (_currentInputField.textComponent != null)
            _currentInputField.textComponent.ForceMeshUpdate();
        Canvas.ForceUpdateCanvases();
        _currentInputField.onValueChanged?.Invoke(text);
    }

    private IEnumerator RestoreFocusNextFrame() {
        yield return null;
        _pendingFocusRestore = null;
        RestoreInputFieldFocusImmediately();
    }

    private void RestoreInputFieldFocusImmediately() {
        if (_currentInputField == null || _isRestoringFocus)
            return;

        _isRestoringFocus = true;

        try {
            if (EventSystem.current != null && EventSystem.current.currentSelectedGameObject != _currentInputField.gameObject)
                EventSystem.current.SetSelectedGameObject(_currentInputField.gameObject);

            _currentInputField.Select();
            _currentInputField.ActivateInputField();
            _currentInputField.MoveTextEnd(false);
        }
        finally {
            _isRestoringFocus = false;
        }
    }
}
