namespace Data {
    public struct Container {
        public string type { get; }
        public string timestamp { get; }
        public object content { get; }

        private Container(string type, string timestamp, object payload) {
            this.type = type;
            this.timestamp = timestamp;
            this.content = payload;
        }

        public static Container Of(string type, object payload) {
            var timestamp = "0";
            return new Container(type, timestamp, payload);
        }
    }
}