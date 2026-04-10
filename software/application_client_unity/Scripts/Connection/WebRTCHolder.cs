using System;
using Unity.WebRTC;
using UnityEngine;

namespace Connection {
    public class WebRTCHolder : MonoBehaviour {

        private static bool _running;
        
        private void Awake() {
            if (_running) { Destroy(gameObject); return; }
            _running = true;
            DontDestroyOnLoad(gameObject);

            StartCoroutine(WebRTC.Update());
        }
    }
}