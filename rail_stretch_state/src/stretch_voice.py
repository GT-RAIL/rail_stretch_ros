#!/usr/bin/env python3
import rospy
from std_srvs.srv import Trigger, TriggerResponse
import azure.cognitiveservices.speech as speechSDK
import json
import pyttsx3
from rail_stretch_state.srv import ExecuteStretchMission, ExecuteStretchMissionRequest

class StretchVoice():
    def __init__(self):
        self.synth = pyttsx3.init()
        self.synth.setProperty('volume', 1.0) #default is 1
        self.synth.setProperty('rate', 140) #default is 200
        self.objectStr = "None"
        self.locationStr = "None"
        self.destinationStr = "None"

        input_config = speechSDK.AudioConfig(use_default_microphone=True) #(device_name=str(get_respeaker_device_id()))
        intent_config = speechSDK.SpeechConfig(subscription="0d7b97472e56436da92d7257b8837646", region="westus")
        self.intent_recognizer = speechSDK.intent.IntentRecognizer(speech_config=intent_config, audio_config=input_config)

        model = speechSDK.intent.LanguageUnderstandingModel(app_id="c73fe257-f15e-4532-ae8a-5ffe6640678f")
        self.intent_recognizer.add_all_intents(model)
        rospy.wait_for_service('/stretch_mission')
        rospy.loginfo('Connected to /stretch_mission')
        self.trigger_stretch_mission_service = rospy.ServiceProxy('/stretch_mission', ExecuteStretchMission)
        self.stretch_voice_service = rospy.Service("stretch_voice", Trigger, self.stretch_voice_handler)
        self.aruco_data = rospy.get_param('/aruco_marker_info')
        self.aruco_names = []
        
        for aruco_id, aruco_datum in self.aruco_data.items():
            self.aruco_names.append(aruco_datum['name'])
    
    def stretch_voice_handler(self,request):
        self.repeat = True
        while(self.repeat):
            print("Say something...")
            intent_result = self.intent_recognizer.recognize_once()
            print("Recognizing...")

            heard = False
            intent_id = None
            objects = None
            location = None
            destination = None


            if intent_result.reason == speechSDK.ResultReason.RecognizedIntent:
                heard = True
                intent_id = str(intent_result.intent_id)

                if str(intent_result.intent_id) == 'MoveObject':
                    info = json.loads(intent_result.intent_json)
                    entityList = info['entities']
                    for i in entityList:
                        eType = str(i['type'])
                        childEntity = str(i['children'][0]['entity'])
                        if eType == 'Object Phrase' and objects == None:
                            objects = childEntity
                        elif eType == 'Location Phrase' and location == None:
                            location = childEntity
                        elif eType == 'Destination Phrase' and destination == None:
                            destination = childEntity

            elif intent_result.reason == speechSDK.ResultReason.RecognizedSpeech:
                heard = True
            elif intent_result.reason == speechSDK.ResultReason.NoMatch:
                pass

            if not heard:
                self.synth.say("I did not hear any command.")
                self.repeat = False
            elif intent_id == None:
                self.synth.say("I did not recognize any command intent.")
            elif intent_id == 'None':
                self.synth.say("That is not a valid command.")
            elif intent_id == 'MoveObject':
                self.objectStr = "None"
                self.locationStr = "None"
                self.destinationStr = "None"

                if not objects == None:
                    self.objectStr = str(objects)
                if not location == None:
                    self.locationStr = str(location)
                if not destination == None:
                    self.destinationStr = str(destination)

                print("Recognized text: " + str(intent_result.text))
                print("Intent: " + intent_id)
                print("Object: " + self.objectStr)
                print("Location: " + self.locationStr)
                print("Destination: " + self.destinationStr)

                if objects == None:
                    self.synth.say("I did not hear an object to move.")
                elif location == None:
                    self.synth.say("I did not hear the location of the " + self.objectStr)
                elif destination == None:
                    self.synth.say("I did not hear where you want me to put the " + self.objectStr)
                elif destination not in self.aruco_names:
                    self.synth.say("I did not see the destination before")
                elif location not in self.aruco_names:
                    self.synth.say("I did not see the location before")
                elif objects not in self.aruco_names:
                    self.synth.say("I don't have " + + self.objectStr + " in my database")
                else:
                    self.repeat = False
                    self.synth.say("I will bring the " + self.objectStr + " from the " + self.locationStr + " to the " + self.destinationStr)

            if self.repeat:
                self.synth.say("Please try again")
            self.synth.runAndWait()
        
        mission_request = ExecuteStretchMissionRequest()
        mission_request.from_aruco_name = self.locationStr
        mission_request.object_name = self.objectStr
        mission_request.to_aruco_name = self.destinationStr
        result = self.trigger_stretch_mission_service(mission_request)
        if result.mission_success == True:
            return TriggerResponse(
            success=True,
            message='Completed successfully with voice input'
            )
        else:
            return TriggerResponse(
            success=False,
            message='Failed with voice input'
            )
        
        self.synth.stop()



if __name__ == '__main__':
    try:
        rospy.init_node('stretch_voice')
        node = StretchVoice()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo('shutting down')