using System;

namespace Events.Policies.Errors {
    public class SwallowErrorPolicy : IErrorPolicyHandler {
        public ErrorPolicyResult ExecuteHandle(Action executable) {
            try {
                executable();
            } catch (Exception ex) {
                ErrorPolicyResult result = new ErrorPolicyResult {
                    Exception = ex,
                    Handled = false,
                    Message = ex.Message,
                    StackTrace = ex.StackTrace
                };
                return result;
            }

            return new ErrorPolicyResult {
                Handled = true,
                Message = "No Error",
                Exception = null,
                StackTrace = ""
            };
        }
    }
}