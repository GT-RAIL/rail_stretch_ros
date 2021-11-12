import azure.cognitiveservices.speech as speechSDK
import json
import pyttsx3

synth = pyttsx3.init()
synth.setProperty('volume', 0.2) #default is 1
synth.setProperty('rate', 140) #default is 200

input_config = speechSDK.AudioConfig(use_default_microphone=True) #(device_name=str(get_respeaker_device_id()))
intent_config = speechSDK.SpeechConfig(subscription="0d7b97472e56436da92d7257b8837646", region="westus")
intent_recognizer = speechSDK.intent.IntentRecognizer(speech_config=intent_config, audio_config=input_config)

model = speechSDK.intent.LanguageUnderstandingModel(app_id="c73fe257-f15e-4532-ae8a-5ffe6640678f")
intent_recognizer.add_all_intents(model)

repeat = True

while(repeat):
    print("Say something...")
    intent_result = intent_recognizer.recognize_once()
    print("Recognizing...")

    heard = False
    intent_id = None
    object = None
    location = None
    destination = None


    if intent_result.reason == speechSDK.ResultReason.RecognizedIntent:
        #print("Recognized: \"{}\" with intent id `{}`".format(intent_result.text, intent_result.intent_id))

        heard = True
        intent_id = str(intent_result.intent_id)

        if str(intent_result.intent_id) == 'MoveObject':
            info = json.loads(intent_result.intent_json)
            #print(str(info))
            entityList = info['entities']
            for i in entityList:
                eType = str(i['type'])
                childEntity = str(i['children'][0]['entity'])
                if eType == 'Object Phrase' and object == None:
                    object = childEntity
                elif eType == 'Location Phrase' and location == None:
                    location = childEntity
                elif eType == 'Destination Phrase' and destination == None:
                    destination = childEntity

    elif intent_result.reason == speechSDK.ResultReason.RecognizedSpeech:
        #print("Recognized: {}".format(intent_result.text))
        heard = True
    elif intent_result.reason == speechSDK.ResultReason.NoMatch:
        #print("No Match for " + intent_result.text + " :: " + str(intent_result.no_match_details.reason))
        pass

    if not heard:
        synth.say("I did not hear any command.")
        repeat = False
    elif intent_id == None:
        synth.say("I did not recognize any command intent.")
    elif intent_id == 'None':
        synth.say("That is not a valid command.")
    elif intent_id == 'MoveObject':
        objectStr = "None"
        locationStr = "None"
        destinationStr = "None"

        if not object == None:
            objectStr = str(object)
        if not location == None:
            locationStr = str(location)
        if not destination == None:
            destinationStr = str(destination)

        print("Recognized text: " + str(intent_result.text))
        print("Intent: " + intent_id)
        print("Object: " + objectStr)
        print("Location: " + locationStr)
        print("Destination: " + destinationStr)

        if object == None:
            synth.say("I did not hear an object to move.")
        elif location == None:
            synth.say("I did not hear the location of the " + objectStr)
        elif destination == None:
            synth.say("I did not hear where you want me to put the " + objectStr)
        else:
            repeat = False
            synth.say("I will bring the " + objectStr + " from the " + locationStr + " to the " + destinationStr)

    if repeat:
        synth.say("Please try again.")
    synth.runAndWait()
    synth.stop()
