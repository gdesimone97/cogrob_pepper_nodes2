import httpx
import os

token = os.environ["GITHUB_TOKEN"]


url = "https://models.github.ai/inference/chat/completions"
headers = {
    "Content-Type": "application/json",
    "Authorization": "Bearer {}".format(token)
}

data = {
    "messages": [
        {
            "role": "system",
            "content": "You are a helpful assistant."
        },
        {
            "role": "user",
            "content": "What is the capital of France?"
        }
    ],
    "temperature": 1.0,
    "top_p": 1.0,
    "max_tokens": 1000,
    "model": "mistral-ai/mistral-medium-2505"
}

response = httpx.post(url, headers=headers, json=data)

print(response.json())
