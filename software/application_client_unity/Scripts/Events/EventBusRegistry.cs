using System;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;
using Utils;

namespace Events {
    public static class EventBusRegistry {
        private static IReadOnlyList<Type> EventTypes { get; set; }
        private static IReadOnlyList<Type> EventBusTypes { get; set; }

#if UNITY_EDITOR
        public static PlayModeStateChange PlayModeState { get; set; }

        [InitializeOnLoadMethod]
        public static void InitializeEditor() {
            EditorApplication.playModeStateChanged -= OnPlayModeStateChanged;
            EditorApplication.playModeStateChanged += OnPlayModeStateChanged;
        }

        static void OnPlayModeStateChanged(PlayModeStateChange state) {
            PlayModeState = state;
            if (state == PlayModeStateChange.ExitingPlayMode) {
                ClearAllBuses();
            }
        }
#endif

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
        public static void Initialize() {
            EventTypes = PredefinedAssemblyUtil.GetTypes(typeof(IEvent));
            EventBusTypes = InitializeAllBuses();
        }

        private static IReadOnlyList<Type> InitializeAllBuses() {
            List<Type> eventBusTypes = new List<Type>();

            var typedef = typeof(EventBus<>);
            foreach (var type in EventTypes) {
                Debug.Log($"Initializing Event Bus for Event {type.Name}");
                var busType = typedef.MakeGenericType(type);
                eventBusTypes.Add(busType);
#if ENABLE_DEBUG && DEBUG_EVENTS
                    Debug.Log($"Initialized Event Bus for Event {type.Name}");
#endif
            }

            return eventBusTypes;
        }

        public static void ClearAllBuses() {
#if ENABLE_DEBUG && DEBUG_EVENTS
                Debug.Log("Clearing all buses");
#endif
            for (int i = 0; i < EventBusTypes.Count; i++) {
                var busType = EventBusTypes[i];
                var clearMethod = busType.GetMethod("Clear",
                    System.Reflection.BindingFlags.Static | System.Reflection.BindingFlags.Public);
                clearMethod.Invoke(null, null);
            }
        }
    }
}