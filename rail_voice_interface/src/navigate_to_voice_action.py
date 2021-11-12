import azure.cognitiveservices.speech as speechSDK
import json


class NavigateToVoiceAction:
#print("Say something...")
  def __init__(self):
    input_config = speechSDK.AudioConfig(use_default_microphone=True)
    intent_config = speechSDK.SpeechConfig(subscription="0d7b97472e56436da92d7257b8837646", region="westus")
    intent_recognizer = speechSDK.intent.IntentRecognizer(speech_config=intent_config, audio_config=input_config)

    model = speechSDK.intent.LanguageUnderstandingModel(app_id="c73fe257-f15e-4532-ae8a-5ffe6640678f")
    intent_recognizer.add_all_intents(model)

  def interpretResult(intent_result):
#intent_result = intent_recognizer.recognize_once()
#print("Recognizing...")

    object = None
    location = None
    destination = None
    intent_id = None
    heard = False

    if intent_result.reason == speechSDK.ResultReason.RecognizedIntent:
      print("Recognized: \"{}\" with intent id `{}`".format(intent_result.text, intent_result.intent_id))
      intent_id = intent_result.intent_id
      heard = True
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
#      print("Recognized: {}".format(intent_result.text))
      heard = True
    elif intent_result.reason == speechSDK.ResultReason.NoMatch:
#      print("No Match for " + intent_result.text + " :: " + str(intent_result.no_match_details.reason))
      pass
    return [heard, intent_id, object, location, destination]

  def send_voice_command:
    print("Say something...")
    intent_result = intent_recognizer.recognize_once()
    print("Recognizing...")
    result = interpretResult(intent_result)

    if result[0] == None:
      print("Nothing was heard by the microphone")
    elif result[1] = None:
      print("There was no recognized result")
    elif result[2] = None:
      print("The recognized intent was 'None'")
    else:
      pass #send ros service call with location and destination passed through

if __name__ = = '__main__':
#  rospy.init_node('navigate_to_voice_action')
  navigate_to_voice_action = NavigateToVoiceAction()
#  rospy.spin()
