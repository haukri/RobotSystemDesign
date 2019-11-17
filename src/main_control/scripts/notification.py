import json
import httplib

def sendPushNotification(message):
    connection = httplib.HTTPSConnection('api.pushed.co', 443)
    connection.connect()
    connection.request('POST', '/1/push', json.dumps({
        "app_key": "SSSLglASNSgdN3Feakul",
        "app_secret": "uUheJ69kVGYdF4eGwKqk6o064YA2NWBpKVbFe7WyKFUWtW0xcrs8QyPnQfVjkZU8",
        "target_type": "app",
        "content": message}),
        {
            "Content-Type": "application/json"
        }
    )

