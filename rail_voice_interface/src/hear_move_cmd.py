import os
import sys
import azure.cognitiveservices.speech as speechSDK
import json
import pyaudio
from contextlib import contextmanager

#def get_respeaker_device_id():
#    with ignore_stderr():
#        p = pyaudio.PyAudio()
#    info = p.get_host_api_info_by_index(0)
#    num_devices = info.get('deviceCount')

#    device_id = -1
#    for i in range(num_devices):
#        if (p.get_device_info_by_host_api_device_index(0, i).get('maxInputChannels')) > 0:
#            if "ReSpeaker" in p.get_device_info_by_host_api_device_index(0, i).get('name'):
#                device_id = i

#    return device_id

#@contextmanager
#def ignore_stderr():
#    devnull = None
#    try:
#        devnull = os.open(os.devnull, os.O_WRONLY)
#        stderr = os.dup(2)
#        sys.stderr.flush()
#        os.dup2(devnull, 2)
#        try:
#            yield
#        finally:
#            os.dup2(stderr, 2)
#            os.close(stderr)
#    finally:
#        if devnull is not None:
#            os.close(devnull)

print("Say something...")

input_config = speechSDK.AudioConfig(use_default_microphone=True) #(device_name=str(get_respeaker_device_id()))
intent_config = speechSDK.SpeechConfig(subscription="0d7b97472e56436da92d7257b8837646", region="westus")
intent_recognizer = speechSDK.intent.IntentRecognizer(speech_config=intent_config, audio_config=input_config)

model = speechSDK.intent.LanguageUnderstandingModel(app_id="c73fe257-f15e-4532-ae8a-5ffe6640678f")
intent_recognizer.add_all_intents(model)

intent_result = intent_recognizer.recognize_once()
print("Recognizing...")


if intent_result.reason == speechSDK.ResultReason.RecognizedIntent:
    print("Recognized: \"{}\" with intent id `{}`".format(intent_result.text, intent_result.intent_id))
    object = None
    location = None
    destination = None

    if str(intent_result.intent_id) == 'MoveObject':
        info = json.loads(intent_result.intent_json)
        entityList = info['entities']
        object = entityList[0]['children'][0]['children'][0]['entity']
        location = entityList[0]['children'][1]['children'][0]['entity']
        destination = entityList[0]['children'][2]['children'][0]['entity']

    objectStr = "None"
    locationStr = "None"
    destinationStr = "None"
    if not object == None:
        objectStr = str(object)
    if not location == None:
        locationStr = str(location)
    if not destination == None:
        destinationStr = str(destination)

    print("Object: " + objectStr + "\nLocation: " + locationStr + "\nDestination: " + destinationStr)
elif intent_result.reason == speechSDK.ResultReason.RecognizedSpeech:
    print("Recognized: {}".format(intent_result.text))
elif intent_result.reason == speechSDK.ResultReason.NoMatch:
    print("No Match for " + intent_result.text + " :: " + str(intent_result.no_match_details.reason))
