from flask import Flask, request, jsonify
import requests, os

app = Flask(__name__)

OPENAI_KEY = os.getenv("OPENAI_KEY")
OPENAI_URL = "https://api.openai.com/v1/chat/completions"

@app.route("/ai", methods=["POST"])
def ai():
    d = request.json
    prompt = f"""
You are an IoT warning assistant.
Temperature: {d['temp']} Celsius
Humidity: {d['hum']}%
Gas: {d['gas']}%
Noise: {d['noise']} dB

Thresholds:
- Temperature >= 32
- Humidity not in 40�70
- Gas > 10
- Noise > 85 dB

Explain what is dangerous and what the user should do.
Keep answer short.
"""

    res = requests.post(
        OPENAI_URL,
        headers={"Authorization": f"Bearer {OPENAI_KEY}"},
        json={"model":"gpt-4o-mini","messages":[{"role":"user","content":prompt}]}
    )

    return jsonify({"reply": res.json()["choices"][0]["message"]["content"]})

app.run(host="0.0.0.0", port=8000)
