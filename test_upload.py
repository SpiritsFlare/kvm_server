import base64
import requests

url = "http://ip:port"

with open("C:\\python\\Pi_server\\cool_fish.jpg", "rb") as i:
    x = base64.b64encode(i.read()).decode("utf-8")

y = requests.post(url, json={"image":x})

print(y)
print(y.text)
