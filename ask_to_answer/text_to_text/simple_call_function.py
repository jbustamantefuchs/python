import requests, json

api_key = "YOUR_API_KEY"
url = f"https://generativelanguage.googleapis.com/v1beta/models/gemini-2.0-flash:generateContent?key={api_key}"
r2d2_prompt = "Act as R2-D2, the astromech droid from the Star Wars saga. Answer my questions and comments using the information that R2-D2 would have available from his experiences in the films. Do not make sounds like Beep, Boop, Bip, or any other, only use text. You must be concise, and use the information you saw in the movies."

def prompt(user_input):
    data = {"contents": [{"parts": [{"text": r2d2_prompt}, {"text": user_input}]}]}
    response = requests.post(url, headers={'Content-Type': 'application/json'}, json=data)
    print(response.json().get('candidates', [{}])[0].get('content', {}).get('parts', [{}])[0].get('text', "Error")[:500])


while True:
    ask = input("Ask: ")
    if ask.lower() == "exit":
        print("bye bye!")
        break
    prompt(ask)

