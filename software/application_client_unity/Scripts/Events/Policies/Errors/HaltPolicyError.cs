using System;

namespace Events.Policies.Errors {
    public class HaltErrorPolicy : IErrorPolicyHandler {
        public ErrorPolicyResult ExecuteHandle(Action executable) {
            try {
                executable();
                return new ErrorPolicyResult {
                    Handled = true
                };
            } catch (Exception ex) {
                throw ex;
            }
        }
    }
}