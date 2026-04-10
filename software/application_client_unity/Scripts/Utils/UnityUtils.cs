

namespace Utils {
    public static class UnityUtils {

    public static float NormalizeAngle(float angle) {
        return angle > 180f ? angle - 360f : angle;
    }
}
}