using System;
using System.Security.Cryptography;
using System.Text;

namespace utils
{
    public static class HMACEncoder {

        private static HMACSHA256 _encoder;

        public static void SetEncoder(string secret) {
            if (_encoder == null) {
                _encoder = new HMACSHA256();
            }
            _encoder.Key = Encoding.UTF8.GetBytes(secret);
        }
        
        public static string GenerateAnswer(string deviceId, string nonce, long timestamp) {
            if (_encoder == null)
                throw new TypeInitializationException($"Encoder is not initialized. Use {nameof(SetEncoder)} first.",
                    new NullReferenceException("Encoder is null"));
            var message = $"{deviceId}|{nonce}|{timestamp}";
            var hash = _encoder.ComputeHash(Encoding.UTF8.GetBytes(message));
            var sb = new StringBuilder(hash.Length * 2);
            foreach (var b in hash)
                sb.Append(b.ToString("x2"));
            return sb.ToString();
        }
    }
}