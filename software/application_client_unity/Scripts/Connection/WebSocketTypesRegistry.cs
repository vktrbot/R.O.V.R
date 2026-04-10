using System.Collections.Generic;
using Connection.Types;
using UnityEngine;

namespace Connection {
    public static class WebSocketTypesRegistry {
        private static Dictionary<string, IWebSocketTypeHandler> _registry = new  Dictionary<string, IWebSocketTypeHandler>();

        public static void RegisterHandler(string type, IWebSocketTypeHandler handler) {
            if (_registry.ContainsKey(type)) {
                Debug.LogError($"{type} is already registered");
                return;
            }
            _registry.Add(type, handler);
        }

        public static void UnregisterHandler(string type) {
            _registry.Remove(type);
        }

        public static object Handle(string type, Dictionary<string, object> args) {
            if (!_registry.ContainsKey(type)) {
                Debug.LogError($"{type} is not registered");
                return null;
            }
            var result = _registry[type].Handle(args);
            if (result.Status == HandlerStatus.ERR) {
                Debug.LogError($"{type} is not handled. Message: {result.ErrorMessage}");
                return null;
            }

            return result.Answer;
        }
    }
}