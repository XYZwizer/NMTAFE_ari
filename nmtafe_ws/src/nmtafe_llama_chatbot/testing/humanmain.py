#!/usr/bin/env python3
# -*- coding: utf-8 -*-
print("importing lib")
from chat import ChatModel
chat = ChatModel()
print("done importing lib")

while True:
    message = input("You: ")
    print()
    if message == "exit":
        break
    print("Bot:", chat.message(message))
    print() 