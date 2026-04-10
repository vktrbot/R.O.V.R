namespace Connection.Types {
    public class Payload {
        public string type;
        public object content;

        public Payload(string type, object content) {
            this.type = type;
            this.content = content;
        }
    }
}