using System;
using Connection.Types;
using Events;
using Events.Types;
using MikeSchweitzer.WebSocket;
using Newtonsoft.Json;
using UnityEngine;
using utils;
using WebSocketState = MikeSchweitzer.WebSocket.WebSocketState;

[Serializable]
public struct CandidateMessage {
    public string candidate;
    public string sdpMid;
    public int sdpMLineIndex;
}

namespace Connection {
    public class WebConnector : MonoBehaviour {

        private WebSocketConnection _socket;

        public bool IsConnected => _socket.State == WebSocketState.Connected;

        private Listener<ConnectSocketServerEvent> _onConnect;
        private Listener<JoinLobbyEvent> _onLobbyConnect;
        private Listener<OnIceCandidateEvent> _onIceCandidate;

        void Start() {
            _socket =  GetComponent<WebSocketConnection>();
            _socket.StateChanged += HandleStateChange;
            _socket.MessageReceived += HandleMessage;
            WebSocketTypesRegistry.RegisterHandler("challenge", new ChallengeTypeHandler());
            WebSocketTypesRegistry.RegisterHandler("auth_ok", new AuthTypeHandler());
            WebSocketTypesRegistry.RegisterHandler("offer", new OfferTypeHandler());
            WebSocketTypesRegistry.RegisterHandler("client_candidate", new CandidateTypeHandler());


            _onConnect = new Listener<ConnectSocketServerEvent>(OnConnect);
            EventBus<ConnectSocketServerEvent>.Register(_onConnect, this);
            _onLobbyConnect = new Listener<JoinLobbyEvent>(OnJoinLobby);
            EventBus<JoinLobbyEvent>.Register(_onLobbyConnect, this);
            _onIceCandidate = new Listener<OnIceCandidateEvent>(OnCandidate);
            EventBus<OnIceCandidateEvent>.Register(_onIceCandidate, this);
        }

        private void OnCandidate(OnIceCandidateEvent e) {
            CandidateMessage message = new CandidateMessage {
                candidate = e.candidate,
                sdpMid = e.sdpMid,
                sdpMLineIndex = e.sdpMLineIndex
            };
            var payload = new Payload("candidate", message);
            _socket.AddOutgoingMessage(JsonConvert.SerializeObject(payload));
        }

        private void OnJoinLobby(JoinLobbyEvent e) {
            _socket.AddOutgoingMessage($"{{\"type\": \"connect_lobby\", \"id\": \"{e.LobbyID}\"}}");
        }

        private void OnConnect(ConnectSocketServerEvent e) {
            var message = e.Secret;
            HMACEncoder.SetEncoder(message);
            _socket.Connect("##your ip here##");
        }

        private void HandleMessage(WebSocketConnection connection, WebSocketMessage message) {
            var data = message.String;
            var json = JsonUtils.Parse(data);
            var type = json["type"] as string;
            var answer = WebSocketTypesRegistry.Handle(type, json);

            if (answer is INetworkSerializable result) {
                var payload = result.Serialize();
                connection.AddOutgoingMessage(payload);
            }

            if (answer is IUnityWorker worker) {
                var go = new GameObject();
                Instantiate(go);
                StartCoroutine(worker.Work(go, connection));
            }
        }

        private void HandleStateChange(WebSocketConnection connection, WebSocketState oldState, WebSocketState newState) {
            ConnectEvent e = new ConnectEvent(ConnectEventType.DISCONNECTED);
            if (newState.Equals(WebSocketState.Connected)) {
                Debug.Log("WebSocket connected");
                e.Type = ConnectEventType.CONNECTED;
            }

            if (newState.Equals(WebSocketState.Disconnected)) {
                Debug.Log("WebSocket disconnected");
                e.Type = ConnectEventType.DISCONNECTED;
            }

            if (newState.Equals(WebSocketState.Connecting)) {
                e.Type = ConnectEventType.CONNECTING;
            }

            if (newState.Equals(WebSocketState.Disconnecting)) {
                e.Type = ConnectEventType.DISCONNECTING;
            }

            EventBus<ConnectEvent>.Raise(e);
        }

        private void HandleError(string errorMsg) {
            Debug.LogError("Cannot connect to the server: " + errorMsg);
        }

    }
}
