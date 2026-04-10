using System.Collections;
using System.Collections.Generic;
using JetBrains.Annotations;
using MikeSchweitzer.WebSocket;
using UnityEngine;

namespace Connection.Types {
    public interface IWebSocketTypeHandler {
        HandlerResult Handle(Dictionary<string, object> args);
    }

    public interface INetworkSerializable {
        string Serialize();
    }

    public interface IUnityWorker {
        IEnumerator Work(GameObject go, WebSocketConnection socket);
    }

    public enum HandlerStatus {
        OK,
        ERR
    }

    public struct HandlerResult {
        public bool IsHandled;
        [CanBeNull] public string ErrorMessage;
        public HandlerStatus Status;
        public object Answer;
    }
}