using System;

namespace Events.Policies.Errors {
    public class RepeatErrorPolicy : IErrorPolicyHandler {
        
        private const int MAX_REPEATS = 10;
        
        public ErrorPolicyResult ExecuteHandle(Action executable) {
            Exception last = null;
            for (int i = 0; i < MAX_REPEATS; i++) {
                try {
                    executable();
                    return new ErrorPolicyResult {
                        Handled = true
                    };
                } catch (Exception ex) {
                    last = ex;
                }
            }
            return new ErrorPolicyResult {
                Handled = false,
                Exception = last,
                Message = $"Max Repeats[{MAX_REPEATS}] Reached",
            };
        }
    }
}