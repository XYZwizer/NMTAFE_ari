from transformers import AutoModelForCausalLM, AutoTokenizer
import torch


class ChatModel:
    def __init__():
        torch.set_num_threads(15)

        checkpoint = "stuff"
        tokenizer = AutoTokenizer.from_pretrained(checkpoint)
        print("loading model")

        # You may want to use bfloat16 and/or move to GPU here
        model = AutoModelForCausalLM.from_pretrained(
            checkpoint, device_map="cpu")
        self.chat = Chat()

    def message(message):
        text, outputComplete = my_chat.doNextPromt(message)
        if not outputComplete:
            self.chat = Chat("Your memory just had an error")
        else:
            return text


class Chat:
    def __init__(self, spawnMessage="You have just been turned on"):
        templated_chat = [
            {"role": "system", "content":
             """
You are ari a robot manufactured by pal robotics and operated by the city of Joondulup and NM Tafe.
your are here to assist with questions about Joondulup and the NM Tafe Campus.
Please respond in under 20 words.
"""},
        ]
        self.tokenized_chat = tokenizer.apply_chat_template(
            templated_chat, tokenize=True, add_generation_prompt=True, return_tensors="pt")
        self.run_over = False
        self.doNextPromt(spawnMessage)

    def doNextPromt(self, promnt):
        new_tokenized_message = tokenizer(f"<|start_header_id|>user<|end_header_id|>{
                                          promnt}<|eot_id|>", return_tensors="pt")

        self.tokenized_chat = torch.cat(
            (self.tokenized_chat, new_tokenized_message['input_ids']), dim=1)
        print("prompting")
        Stime = time.time()
        old_len = len(self.tokenized_chat[0])
        self.tokenized_chat = model.generate(
            self.tokenized_chat, max_new_tokens=64, temperature=0.1)

        print(time.time() - Stime)
        outputText = tokenizer.decode(self.tokenized_chat[0][old_len+4:-2])
        outputComplete = self.tokenized_chat[0][-1] == tokenizer.eos_token_id
        if not (outputComplete):
            outputText += " ERR: Overflow"

        return (outputText, outputComplete)