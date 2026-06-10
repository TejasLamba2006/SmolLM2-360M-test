# pip install accelerate
import torch
from transformers import AutoTokenizer, AutoModelForCausalLM
checkpoint = "HuggingFaceTB/SmolLM2-360M"
tokenizer = AutoTokenizer.from_pretrained(checkpoint)
# for fp16 use `torch_dtype=torch.float16` instead
model = AutoModelForCausalLM.from_pretrained(
    checkpoint, device_map="auto", torch_dtype=torch.float32)
inputs = tokenizer.encode("Gravity is", return_tensors="pt")
outputs = model.generate(inputs)
print(tokenizer.decode(outputs[0]))
