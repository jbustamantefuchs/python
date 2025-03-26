#============ IMPORT ============
import paho.mqtt.client as mqtt, requests, json, random, os, pygame, pyaudio, sounddevice, threading
from time import sleep
from vosk import Model, KaldiRecognizer
import time
#================================

#=========== VARIABLE ===========
#MQTT
broker = "localhost"
port = 1883
topic = "presence"
#

#Val to check presences
last_value = None
last_time = time.time()
is_printed = False
person_detected = False
person_not_detected = False
#

#Val for threads
stop_speak = False
speak_thread = None
#

#Val to set-up PATHS, API, prompt
model_path = "/PATH/TO/MODEL/vosk-model-small-en-us-0.15"
api_key = "YOUR_API_KEY"
url = f"https://generativelanguage.googleapis.com/v1beta/models/gemini-2.0-flash:generateContent?key={api_key}"
r2d2_prompt = "Act as R2-D2, the astromech droid from the Star Wars saga. Answer my questions and comments using the information that R2-D2 would have available from his experiences in the films. Do not make sounds like Beep, Boop, Bip, or any other, only use text. You must be concise, and use the information you saw in the movies."
#
#================================

#====== START PYGAME AUDIO ======
pygame.mixer.init()
#================================

#========== DEF BLOCKS ==========
def speak():#Listen Voice
    global stop_speak

    model = Model(model_path)
    recognizer = KaldiRecognizer(model, 16000)
    p = pyaudio.PyAudio()
    stream = p.open(format=pyaudio.paInt16,
                    channels=1,
                    rate=16000,
                    input=True,
                    frames_per_buffer=4000)
    stream.start_stream()
    print("Start speaking...")
    dicho = ""

    try:
        while not stop_speak:
            data = stream.read(4000, exception_on_overflow=False)
            if recognizer.AcceptWaveform(data):
                result = recognizer.Result()
                result_json = json.loads(result)
                text = result_json.get("text", "").strip()
                if text:
                    dicho = text
                    print(f"Recognized: {dicho}")
                    prompt(dicho)
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("Manually interrumpted")

    finally:
        stream.stop_stream()
        stream.close()
        p.terminate()

def play_random_sound():#Play Audio
    sound_dir = "/PATH/TO/SOUNDS" # CHANGE TO YOUR SOUNDS FOLDER
    sounds = [f for f in os.listdir(sound_dir) if f.endswith(".mp3")]
    if sounds:
        pygame.mixer.music.load(os.path.join(sound_dir, random.choice(sounds)))
        pygame.mixer.music.play()

def random_sound_decision():#Choose Audio
    decision = random.choice([0, 1, 2])
    if decision in [1, 3]:
        play_random_sound()
        sleep(2)
    return decision

def prompt(user_input):#Generate Respond
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

def on_message(client, userdata, msg):#Manage Process & Mqtt Receiver
    global last_value, last_time, is_printed, person_detected, person_not_detected, stop_speak, speak_thread

    try:
        presence_value = int(msg.payload.decode())

        if presence_value != last_value:
            last_value = presence_value
            last_time = time.time()
            is_printed = False  

        if time.time() - last_time >= 5 and not is_printed:
            if presence_value == 1:
                if not person_detected:
                    print("Someone detected")
                    prompt("You detect a friendly person, say a super hi!")
                    stop_speak = False  
                    if speak_thread is None or not speak_thread.is_alive():
                        speak_thread = threading.Thread(target=speak, daemon=True)
                        speak_thread.start()
                    person_detected = True  
                    person_not_detected = False  
                is_printed = True  

            elif presence_value == 0:
                if not person_not_detected:
                    print("Nobody within range")
                    prompt("You don't detect anyone, what would you say?")
                    
                    stop_speak = True  
                    if speak_thread is not None and speak_thread.is_alive():
                        speak_thread.join()
                        speak_thread = None 
                    person_not_detected = True 
                    person_detected = False
                is_printed = True

    except ValueError:
        print("Data no valid")
#================================

#========= MQTT BROKER ==========
client = mqtt.Client()
client.on_message = on_message
client.connect(broker, port, 60)
client.subscribe(topic)
client.loop_start()
#================================

#========== MAIN LOOP ===========
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("Breaking program")
    client.loop_stop()
#================================