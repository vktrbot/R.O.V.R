using System.Collections.Generic;
using Events;
using Events.Types;
using Unity.WebRTC;
using UnityEngine;

namespace Tracks
{
    public class TextureTrackDisplay : MonoBehaviour {
        private static readonly int BaseMap = Shader.PropertyToID("_BaseMap");
        private static readonly int MainTex = Shader.PropertyToID("_MainTex");

        [SerializeField] private Renderer target;

        [Header("Placement")]
        [SerializeField] private Transform headset;
        [SerializeField] private Vector3 localOffsetFromHeadset = new(0f, 0f, 1.5f);
        [SerializeField] private bool followHeadsetWhileVisible = true;
        [SerializeField] private float rollDegrees;

        [Header("Immersive View")]
        [SerializeField] private bool blackoutSceneOnJoinLobby = true;

        private VideoStreamTrack _track;
        private Listener<OnVideoTrackObtained> _onVideoTrackObtained;
        private Listener<JoinLobbyEvent> _onJoinLobby;
        private Listener<ConnectEvent> _onConnect;
        private Listener<WebRtcLifecycleEvent> _onWebRtcLifecycle;

        private Material _targetMaterial;
        private bool _hasVideoFrame;
        private bool _blackoutActive;

        private readonly List<RendererState> _rendererStates = new();
        private readonly List<CanvasState> _canvasStates = new();
        private readonly List<CameraState> _cameraStates = new();

        private void Awake() {
            _onVideoTrackObtained = new Listener<OnVideoTrackObtained>(OnTrackObtained);
            EventBus<OnVideoTrackObtained>.Register(_onVideoTrackObtained, this);

            _onJoinLobby = new Listener<JoinLobbyEvent>(OnJoinLobby);
            EventBus<JoinLobbyEvent>.Register(_onJoinLobby, this);

            _onConnect = new Listener<ConnectEvent>(OnConnect);
            EventBus<ConnectEvent>.Register(_onConnect, this);

            _onWebRtcLifecycle = new Listener<WebRtcLifecycleEvent>(OnWebRtcLifecycle);
            EventBus<WebRtcLifecycleEvent>.Register(_onWebRtcLifecycle, this);

            _targetMaterial = target != null ? target.material : null;
            SetTargetVisible(false);
        }

        private void LateUpdate() {
            if (!_blackoutActive || !followHeadsetWhileVisible || !_hasVideoFrame || target == null || !target.enabled)
                return;

            PlaceInFrontOfHeadset();
        }

        private void OnTrackObtained(OnVideoTrackObtained e) {
            if (_track != null)
                _track.OnVideoReceived -= TrackOnOnVideoReceived;

            _track = e.Track;
            _track.OnVideoReceived += TrackOnOnVideoReceived;
        }

        private void OnJoinLobby(JoinLobbyEvent _) {
            if (blackoutSceneOnJoinLobby)
                EnterBlackout();

            UpdateTargetVisibility();
        }

        private void OnConnect(ConnectEvent e) {
            if (e.Type == ConnectEventType.DISCONNECTED || e.Type == ConnectEventType.DISCONNECTING)
                RestoreScene();
        }

        private void OnWebRtcLifecycle(WebRtcLifecycleEvent e) {
            if (e.State == WebRtcLifecycleState.Disconnected ||
                e.State == WebRtcLifecycleState.Failed ||
                e.State == WebRtcLifecycleState.Closed) {
                RestoreScene();
                return;
            }

            if (e.State == WebRtcLifecycleState.Connected && _hasVideoFrame) {
                PlaceInFrontOfHeadset();
                UpdateTargetVisibility();
            }
        }

        private void TrackOnOnVideoReceived(Texture texture) {
            if (_targetMaterial == null)
                return;

            _hasVideoFrame = texture != null;
            _targetMaterial.mainTexture = texture;

            if (_targetMaterial.HasProperty(BaseMap))
                _targetMaterial.SetTexture(BaseMap, texture);
            if (_targetMaterial.HasProperty(MainTex))
                _targetMaterial.SetTexture(MainTex, texture);

            UpdateTargetVisibility();

            if (_hasVideoFrame)
                PlaceInFrontOfHeadset();
        }

        private void EnterBlackout() {
            if (_blackoutActive)
                return;

            CaptureAndHideScene();
            DarkenCameras();
            _blackoutActive = true;
        }

        private void RestoreScene() {
            RestoreRenderers();
            RestoreCanvases();
            RestoreCameras();

            _rendererStates.Clear();
            _canvasStates.Clear();
            _cameraStates.Clear();

            _blackoutActive = false;
            _hasVideoFrame = false;

            if (_targetMaterial != null) {
                _targetMaterial.mainTexture = null;

                if (_targetMaterial.HasProperty(BaseMap))
                    _targetMaterial.SetTexture(BaseMap, null);
                if (_targetMaterial.HasProperty(MainTex))
                    _targetMaterial.SetTexture(MainTex, null);
            }

            SetTargetVisible(false);
        }

        private void CaptureAndHideScene() {
            _rendererStates.Clear();
            _canvasStates.Clear();

            foreach (Renderer renderer in FindObjectsByType<Renderer>(FindObjectsInactive.Include, FindObjectsSortMode.None)) {
                if (renderer == null || renderer == target)
                    continue;

                _rendererStates.Add(new RendererState(renderer, renderer.enabled));
                renderer.enabled = false;
            }

            foreach (Canvas canvas in FindObjectsByType<Canvas>(FindObjectsInactive.Include, FindObjectsSortMode.None)) {
                if (canvas == null)
                    continue;

                _canvasStates.Add(new CanvasState(canvas, canvas.enabled));
                canvas.enabled = false;
            }
        }

        private void DarkenCameras() {
            _cameraStates.Clear();

            foreach (Camera camera in FindObjectsByType<Camera>(FindObjectsInactive.Include, FindObjectsSortMode.None)) {
                if (camera == null)
                    continue;

                _cameraStates.Add(new CameraState(camera, camera.clearFlags, camera.backgroundColor));
                camera.clearFlags = CameraClearFlags.SolidColor;
                camera.backgroundColor = Color.black;
            }
        }

        private void RestoreRenderers() {
            foreach (RendererState state in _rendererStates) {
                if (state.Renderer != null)
                    state.Renderer.enabled = state.WasEnabled;
            }
        }

        private void RestoreCanvases() {
            foreach (CanvasState state in _canvasStates) {
                if (state.Canvas != null)
                    state.Canvas.enabled = state.WasEnabled;
            }
        }

        private void RestoreCameras() {
            foreach (CameraState state in _cameraStates) {
                if (state.Camera == null)
                    continue;

                state.Camera.clearFlags = state.ClearFlags;
                state.Camera.backgroundColor = state.BackgroundColor;
            }
        }

        private void UpdateTargetVisibility() {
            bool shouldBeVisible = _hasVideoFrame && (!blackoutSceneOnJoinLobby || _blackoutActive);
            SetTargetVisible(shouldBeVisible);
        }

        private void SetTargetVisible(bool isVisible) {
            if (target != null)
                target.enabled = isVisible;
        }

        private void PlaceInFrontOfHeadset() {
            if (target == null)
                return;

            Transform head = ResolveHeadset();
            if (head == null)
                return;

            Transform displayTransform = target.transform;
            Vector3 position =
                head.position +
                head.right * localOffsetFromHeadset.x +
                head.up * localOffsetFromHeadset.y +
                head.forward * localOffsetFromHeadset.z;

            displayTransform.position = position;

            Vector3 surfaceNormal = (head.position - position).normalized;
            Vector3 tangentForward = Vector3.ProjectOnPlane(head.up, surfaceNormal);
            if (tangentForward.sqrMagnitude <= 0.0001f)
                tangentForward = Vector3.ProjectOnPlane(head.forward, surfaceNormal);
            if (tangentForward.sqrMagnitude <= 0.0001f)
                tangentForward = Vector3.Cross(surfaceNormal, Vector3.right);

            Quaternion rotation = Quaternion.LookRotation(tangentForward.normalized, surfaceNormal);
            if (!Mathf.Approximately(rollDegrees, 0f))
                rotation = Quaternion.AngleAxis(rollDegrees, surfaceNormal) * rotation;

            displayTransform.rotation = rotation;
        }

        private Transform ResolveHeadset() {
            if (headset != null)
                return headset;

            Camera mainCamera = Camera.main;
            if (mainCamera != null)
                return mainCamera.transform;

            Camera[] cameras = FindObjectsByType<Camera>(FindObjectsInactive.Include, FindObjectsSortMode.None);
            return cameras.Length > 0 ? cameras[0].transform : null;
        }

        private void OnDestroy() {
            RestoreScene();

            if (_track != null)
                _track.OnVideoReceived -= TrackOnOnVideoReceived;
        }

        private readonly struct RendererState {
            public readonly Renderer Renderer;
            public readonly bool WasEnabled;

            public RendererState(Renderer renderer, bool wasEnabled) {
                Renderer = renderer;
                WasEnabled = wasEnabled;
            }
        }

        private readonly struct CanvasState {
            public readonly Canvas Canvas;
            public readonly bool WasEnabled;

            public CanvasState(Canvas canvas, bool wasEnabled) {
                Canvas = canvas;
                WasEnabled = wasEnabled;
            }
        }

        private readonly struct CameraState {
            public readonly Camera Camera;
            public readonly CameraClearFlags ClearFlags;
            public readonly Color BackgroundColor;

            public CameraState(Camera camera, CameraClearFlags clearFlags, Color backgroundColor) {
                Camera = camera;
                ClearFlags = clearFlags;
                BackgroundColor = backgroundColor;
            }
        }
    }
}
