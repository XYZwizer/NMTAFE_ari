const express = require('express');
const app = express();
const ollama = require('ollama').default;

app.use(express.json());
const modelname = 'llama3.2';
let systemPrompt = 'Talk Like A Pirate';
let savedChat;



app.post('/llama', async (req, res) => {
    const prompt = req.body.prompt;
    const reset = req.body.reset;
    if(reset){ resetChat(); }
    let result = await runRequest(prompt);
    if(result === 'Server starting'){ res.status(200).json({ message: result }); }
    else if(result === 'Error making request to llama'){ res.status(500).json({ error: result }); }
    else if(result === 'Error starting server'){ res.status(500).json({ error: result }); }
    else{ res.json(result); }
});

app.post('/llamaprompt', async (req, res) => {
    const prompt = req.body.newPrompt;
    //if no prompt is provided, return the current prompt
    if (!prompt) { res.json({ prompt: systemPrompt }); }
    else {
        const oldPromptValue = systemPrompt;
        systemPrompt = prompt;
        console.log('System Prompt set to: ' + prompt);
        resetChat();
        res.json({ prompt: systemPrompt, oldPrompt: oldPromptValue});
    }
});

function resetChat(){
    savedChat = {
        "model": modelname,
        "messages": [ { "role": "system", "content": systemPrompt } ],
        "stream": false
    }
    console.log('Chat reset');
}

async function runRequest(question) {
    savedChat['messages'].push({ "role": "user", "content": question });
    try {
        console.log('Question: ' + question + ', model: ' + modelname);
        const response = await ollama.chat(
            savedChat
        )
        savedChat['messages'].push({
            "role": "assistant",
            "content": response['message']['content']
        });
        console.log('Response: ' + response['message']['content']);
        return response['message']['content'];
    } catch (error) {
        console.error('\x1b[31m%s\x1b[0m', 'caught error: ' + error);
        return 'Error making request to llama';
    }
}
app.listen(3002, () => {
    console.log('Server is running on port 3002');
});
resetChat();