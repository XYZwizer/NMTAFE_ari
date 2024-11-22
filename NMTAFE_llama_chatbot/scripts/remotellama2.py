import requests
import time

class LlamaInterface:
    def __init__(self, url="localhost"):
        self.url = "http://" + url + ":3002/"
        self.headers = {"Content-Type": "application/json"}

    def query(self, question, reset=False):
        data = {"prompt": question, "reset": reset}
        response = requests.post(self.url + 'llama', headers=self.headers, json=data)
        response_data = response.json()
        # Check if the response is a dictionary before trying to access it
        if isinstance(response_data, dict):
            #if the response message is "server starting"
            if response_data.get("message") == "Server starting":
                print("Server is starting, please hold...")
                #wait 2 seconds then try again
                time.sleep(2)
                return self.query(question)
            elif response.status_code == 500:
                return response_data.get("error")
            else:
                return response_data.get("message")
        else:
            return response_data  # Return the response as is if it's not a dictionary

    def set_system_prompt(self, prompt):
        if not prompt:
            return "Prompt cannot be empty"
        else:
            data = {"newPrompt": prompt}
            response = requests.post(self.url + 'llamaprompt', headers=self.headers, json=data)
            response_data = response.json()
            # Check if the response is a dictionary before trying to access it
            if isinstance(response_data, dict):
                #if the data was none or empty return the response
                if not data:
                    return response_data.get("prompt")
                else:
                    return [response_data.get("prompt"), response_data.get("oldPrompt")]
            else:
                return response_data

