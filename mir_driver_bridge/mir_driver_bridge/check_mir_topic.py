import websocket
import json

# Your Robot's IP
IP = "10.38.11.17" 

def on_open(ws):
    print(f"Connected to {IP}. Asking for topic list...")
    # We use the 'rosapi' service which is standard on MiR robots
    req = {
        "op": "call_service",
        "service": "/rosapi/topics",
        "args": {}
    }
    ws.send(json.dumps(req))

def on_message(ws, message):
    try:
        data = json.loads(message)
        # Look for the service response
        if 'values' in data and 'topics' in data['values']:
            print("\n=== AVAILABLE INTERNAL TOPICS ===")
            topics = sorted(data['values']['topics'])
            for t in topics:
                # Highlight the ones we care about
                if 'odom' in t or 'scan' in t or 'tf' in t:
                    print(f" -> {t}")
                else:
                    print(t)
            print("\n=================================")
            ws.close()
        else:
            # Sometimes we get other messages first
            pass
    except Exception as e:
        print(f"Error: {e}")

ws = websocket.WebSocketApp(f"ws://{IP}:9090", on_open=on_open, on_message=on_message)
ws.run_forever()