using UnityEngine;

namespace Data {
    public struct LocomotionData {
        public float power;
        public Vector direction;
    }

    public struct Vector {
        public float x;
        public float y;

        public Vector(float x, float y) {
            this.x = x;
            this.y = y;
        }
    }
}