import requests
import json
import time

class LlamaInterface:
    url = "http://localhost:3002/"
    headers = {"Content-Type": "application/json"}
    def query(question, reset=False):
        data = {"prompt": question, "reset": reset}
        response = requests.post(LlamaInterface.url + 'llama', headers=LlamaInterface.headers, json=data)
        response_data = response.json()
        # Check if the response is a dictionary before trying to access it
        if isinstance(response_data, dict):
            #if the response message is "server starting"
            if response_data.get("message") == "Server starting":
                print("Server is starting, please hold...")
                #wait 2 seconds then try again
                time.sleep(2)
                return LlamaInterface.query(question)
            elif response.status_code == 500:
                return response_data.get("error")
            else:
                return response_data.get("message")
        else:
            return response_data  # Return the response as is if it's not a dictionary

    def set_system_prompt(prompt):
        if not prompt:
            return "Prompt cannot be empty"
        else:
            data = {"newPrompt": prompt}
            response = requests.post(LlamaInterface.url + 'llamaprompt', headers=LlamaInterface.headers, json=data)
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

# Example usage
LlamaInterface.set_system_prompt("you are a helpful ai, please answer the following questions in under 40 words each")
question = "what is your opinion on fish?"
print(LlamaInterface.query(question))
print()
question = "how many fish are there?"
print(LlamaInterface.query(question))
print()
question = "what is the best fish?"
print(LlamaInterface.query(question))
print()
question = "what was the second question i asked you?"
print(LlamaInterface.query(question))

