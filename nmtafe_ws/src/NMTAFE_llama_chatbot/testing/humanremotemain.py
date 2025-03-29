#!/usr/bin/env python3
# -*- coding: utf-8 -*-
print("importing lib")
from remotellama2 import LlamaInterface
print("done importing lib")

while True:
    message = input("You: ")
    print()
    if message == "exit":
        break
    print("Bot:", LlamaInterface.query(message))
    print() 