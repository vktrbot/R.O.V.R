using System;

namespace Services {
    public class VrStore {
        private static VrStore _instance;

        public static VrStore Instance {
            get {
                if (_instance == null) {
                    _instance = new VrStore();
                }  
                return _instance;
            }
        }

        private float _power = .1f;
        public float Step { get; set; } = .1f;

        public float Power => _power;

        public string State { get; set; } = "open";
        

        public void IncreasePower() {
            _power = Math.Min(.5f, _power + Step);
        }

        public void DecreasePower() {
            _power = Math.Max(0.1f, _power - Step);
        }
        

    }
}