# mamot ) AGV stage별 음성
import websocket
import json
import winsound
import os
import roslibpy

client = roslibpy.Ros(host='172.30.1.100',port=9090)
client.run()

listener = roslibpy.Topic(client, '/audio_topic','std_msgs/String')
audio_files = {
    "1": r"C:\Users\snug1\Desktop\mamot_proj\audio\1_stage.wav",
    "2": r"C:\Users\snug1\Desktop\mamot_proj\audio\2_stage.wav",  
    "3": r"C:\Users\snug1\Desktop\mamot_proj\audio\3_stage.wav",  
    "4": r"C:\Users\snug1\Desktop\mamot_proj\audio\4_stage.wav",
    "44":r"C:\Users\snug1\Desktop\mamot_proj\audio\44.wav",
    "5": r"C:\Users\snug1\Desktop\mamot_proj\audio\5_stage.wav",
    "6": r"C:\Users\snug1\Desktop\mamot_proj\audio\6_stage.wav"
}

def on_message(message):
    print("Received message:", message)
    signal = message['data']
    if signal in audio_files:
        audio_file = audio_files[signal]
        if os.path.exists(audio_file):
            winsound.PlaySound(audio_file, winsound.SND_FILENAME)
            print(f"Playing audio for signal {signal}")
        else:
            print("Audio file not found!")
    else:
        print("Invalid signal received")

try:
    while True:
        listener.subscribe(on_message)
except KeyboardInterrupt:
    client.terminate()