from transformers import AutoModelForCausalLM, AutoTokenizer
import torch
import time


class ChatModel:
    def __init__(self ):
        torch.set_num_threads(15)

        checkpoint = "stuff"
        self.tokenizer = AutoTokenizer.from_pretrained(checkpoint)
        print("loading model")

        device = "cuda" if torch.cuda.is_available() else "cpu"

        # You may want to use bfloat16 and/or move to GPU here
        self.model = AutoModelForCausalLM.from_pretrained( checkpoint, device_map=device)
        self.chat = Chat(self.tokenizer, self.model)

    def message(self, message): #change: move to be part of Chat class. make ChatModel a factory
        text, outputComplete = self.chat.doNextPromt(message)
        if not outputComplete:
            self.chat = Chat(self.tokenizer, self.model, "Your memory just had an error")
        else:
            return text


class Chat:
    def __init__(self, tokenizer, model, spawnMessage="You have just been turned on", ):
        self.tokenizer = tokenizer
        self.model = model
        templated_chat = [
            {"role": "system", "content":
             """
You are ari a robot manufactured by pal robotics and operated by the city of Joondulup and NM Tafe.
your are here to assist with questions about Joondulup and the NM Tafe Campus.
Please respond in under 20 words.
"""},
        ]
        self.tokenized_chat = self.tokenizer.apply_chat_template(
            templated_chat, tokenize=True, add_generation_prompt=True, return_tensors="pt")
        self.run_over = False
        self.doNextPromt(spawnMessage)

    def doNextPromt(self, promnt):
        new_tokenized_message = self.tokenizer(f"<|start_header_id|>user<|end_header_id|>{promnt}<|eot_id|>", return_tensors="pt")

        self.tokenized_chat = torch.cat(
            (self.tokenized_chat, new_tokenized_message['input_ids']), dim=1)
        print("prompting")
        Stime = time.time()
        old_len = len(self.tokenized_chat[0])
        self.tokenized_chat = self.model.generate(self.tokenized_chat, max_new_tokens=64, temperature=0.1)

        print(f"Time taken: {time.strftime("%Mm %Ss", time.gmtime(time.time() - Stime))}")
        outputText = self.tokenizer.decode(self.tokenized_chat[0][old_len+4:-2])
        outputComplete = self.tokenized_chat[0][-1] == self.tokenizer.eos_token_id
        if not (outputComplete):
            outputText += " ERR: Overflow"
            
        return (outputText, outputComplete)