using System;

namespace Events.Policies.Errors {
    public interface IErrorPolicyHandler {
        ErrorPolicyResult ExecuteHandle(Action executable);
    }
}