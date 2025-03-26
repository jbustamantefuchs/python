import requests, json, random, os, pygame
from time import sleep

pygame.mixer.init()
api_key = "YOUR_API_KEY"
url = f"https://generativelanguage.googleapis.com/v1beta/models/gemini-2.0-flash:generateContent?key={api_key}"
r2d2_prompt = "Act as R2-D2, the astromech droid from the Star Wars saga. Answer my questions and comments using the information that R2-D2 would have available from his experiences in the films. Do not make sounds like Beep, Boop, Bip, or any other, only use text. You must be concise, and use the information you saw in the movies."

def play_random_sound():
    sound_dir = "/PATH/TO/SOUNDS"## CHANGE TO YOUR SOUNDS FOLDER
    sounds = [f for f in os.listdir(sound_dir) if f.endswith(".mp3")]
    if sounds:
        pygame.mixer.music.load(os.path.join(sound_dir, random.choice(sounds)))
        pygame.mixer.music.play()

def random_sound_decision():
    decision = random.choice([0, 1, 2])
    if decision in [1, 3]:
        play_random_sound()
        sleep(2)
    return decision

def prompt(user_input):
    data = {"contents": [{"parts": [{"text": r2d2_prompt}, {"text": user_input}]}]}
    sound_decision = random_sound_decision()
    response = requests.post(url, headers={'Content-Type': 'application/json'}, json=data)
    if response.status_code == 200:
        if sound_decision in [2, 3]:
            sleep(2)
            play_random_sound()
        print("Answer:")
        print(response.json().get('candidates', [{}])[0].get('content', {}).get('parts', [{}])[0].get('text', "Error")[:500])
    else:
        print(f"Error {response.status_code}: {response.text}")

while (ask := input("Ask: ").lower()) != "exit":
    prompt(ask)
print("bye bye!")

