namespace Events {
    public enum ErrorPolicy {
        SWALLOW,
        HALT,
        REPEAT
    }

    public enum Priority {
        LOWEST = 4,
        LOW = 3,
        MEDIUM = 2,
        HIGH = 1,
        HIGHEST = 0,
    }
}