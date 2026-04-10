using System;
using Events;
using Events.Types;
using TMPro;
using Unity.WebRTC;
using UnityEngine;
using UnityEngine.UI;

public class MainMenuHandler : MonoBehaviour {


    [SerializeField] private GameObject _lobby;

    [SerializeField] private Button _connectButton;
    [SerializeField] private Button _connectLobby;

    [SerializeField] private TMP_InputField _secretField;
    [SerializeField] private TMP_InputField _lobbyField;

    private Listener<ConnectEvent> _onConnect;

    private void Awake() {
        _onConnect = new Listener<ConnectEvent>(OnStatusUpdate);
        EventBus<ConnectEvent>.Register(_onConnect, this);

        _connectButton.onClick.AddListener(OnConnect);
        _connectLobby.onClick.AddListener(OnConnectLobby);
        _lobby.SetActive(false);
    }


    private void OnConnectLobby() {
        var lobbyID =  _lobbyField.text;
        JoinLobbyEvent e =  new(lobbyID);
        EventBus<JoinLobbyEvent>.Raise(e);
    }

    private void OnConnect() {
        var secret = _secretField.text;
        ConnectSocketServerEvent e =  new(secret);
        EventBus<ConnectSocketServerEvent>.Raise(e);
    }

    private void OnStatusUpdate(ConnectEvent e) {
        if (e.Type.Equals(ConnectEventType.CONNECTED)) {
            _lobby.SetActive(true);
        } else {
            _lobby.SetActive(false);
        }
    }
    private void OnDestroy() {
        _connectButton.onClick.RemoveAllListeners();
        _connectLobby.onClick.RemoveAllListeners();
    }

}
