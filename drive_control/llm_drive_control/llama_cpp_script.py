# Should be under ASIMOV>src>controller>controller
# Offline LLM llama_cpp to process Natural Language commands and give JSON output

from llama_cpp import Llama,LlamaGrammar
import json

try:
    grammar = LlamaGrammar.from_file("/home/doga/rover_llm/asimov_json_arr.gbnf")     # FIX DIRECTORY
except Exception as e:
    raise RuntimeError(f"Failed to load grammar: {e}")


# GLOBAL VARIABLES
MY_MODEL_PATH = "/home/doga/rover_llm/model/zephyr-7b-beta.Q4_0.gguf"    # FIX DIRECTORY
CONTEXT_SIZE = 512

# LOAD THE MODEL (Zephyr is the model)
zephyr_model = Llama(
    model_path=MY_MODEL_PATH,
    n_ctx=CONTEXT_SIZE,
    
)

def generate_text_from_prompt(user_prompt,
                             max_tokens = 60,
                             temperature = 0.3,
                             top_p = 0.5,
                             echo = False,
                             stop = None
):

   # Define the parameters
   model_output = zephyr_model(
       user_prompt,
       max_tokens=max_tokens,
       temperature=temperature,
       top_p=top_p,
       echo=echo,
       stop=stop,
       grammar=grammar
   )


   return model_output


if __name__ == "__main__":
    while True:
        user_in = input("> ").strip()

        if user_in.lower() in ("q", "quit", "exit"):
            break
        if not user_in:
            continue

        # Force single-line JSON so stop=["\n"] is safe
        prompt = (
            "Translate the user’s instruction into a command JSON matching the grammar. Do not change requested direction/speed/duration."
            f"{user_in}\n"
        )
        zephyr_model_response = generate_text_from_prompt(user_in)
        text = zephyr_model_response["choices"][0]["text"].strip()

        # Print ONLY valid JSON (or error)
        try:
            obj = json.loads(text)
            print(json.dumps(obj, separators=(",", ":")))
        except json.JSONDecodeError as e:
            print(f"JSON decode error: {e}")
            # For debugging if needed, uncomment:
            # print("RAW:", repr(text))


#print("RAW:", repr(text))
#try:
#    obj = json.loads(text)
#    print("PARSED:", json.dumps(obj, indent=2))
#except json.JSONDecodeError as e:
#    print("JSON decode error:", e)
