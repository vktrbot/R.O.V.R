using System.Collections.Generic;
using Newtonsoft.Json;

public static class JsonUtils {
    
    public static Dictionary<string, object> Parse(string str) {
        return JsonConvert.DeserializeObject<Dictionary<string, object>>(str);
    }
}