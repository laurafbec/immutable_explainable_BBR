import os
import logging
import click
import torch

from langchain.evaluation.qa import QAGenerateChain
from langchain.evaluation.qa import QAEvalChain
from langchain.evaluation import load_evaluator

from langchain.evaluation.scoring import ScoreStringEvalChain

from langchain.chat_models import ChatOpenAI


from run_localGPT import retrieval_qa_pipline
from ingest import load_documents

from constants import (
    CHROMA_SETTINGS,
    DOCUMENT_MAP,
    EMBEDDING_MODEL_NAME,
    INGEST_THREADS,
    PERSIST_DIRECTORY,
    SOURCE_DIRECTORY,
	MODELS_PATH,
)

from assisted_evaluation_scn1_fails import eval_examples



criteria = {
    "accuracy": """
    Score 1: The answer is completely unrelated to the reference.
    Score 3: The answer has minor relevance but does not align with the reference.
    Score 5: The answer has moderate relevance but contains inaccuracies.
    Score 7: The answer aligns with the reference but has minor errors or omissions.
    Score 10: The answer is completely accurate and aligns perfectly with the reference.
    """,
    "conciseness": """
    Score 1: The answer is extremely verbose and lacks brevity.
    Score 3: The answer is somewhat wordy and could be more concise.
    Score 5: The answer is moderately concise but could be improved.
    Score 7: The answer is fairly concise and effectively conveys the information.
    Score 10: The answer is highly concise, with no unnecessary words.
    """,
    "coherence": """
    Score 1: The answer is extremely incoherent and lacks structure.
    Score 3: The answer has major issues with coherence and organization.
    Score 5: The answer is somewhat coherent but could be more organized.
    Score 7: The answer is fairly coherent and well-structured.
    Score 10: The answer is highly coherent, with a clear and logical organization.
    """,
    "helpfulness": """
    Score 1: The answer provides no helpful information.
    Score 3: The answer contains minimal helpful information.
    Score 5: The answer is somewhat helpful but lacks depth.
    Score 7: The answer is fairly helpful and provides useful information.
    Score 10: The answer is extremely helpful and offers comprehensive information.
    """
}


# chose device typ to run on as well as to show source documents.
@click.command()
@click.option(
    "--device_type",
    default="cuda" if torch.cuda.is_available() else "cpu",
    type=click.Choice(
        [
            "cpu",
            "cuda",
            "ipu",
            "xpu",
            "mkldnn",
            "opengl",
            "opencl",
            "ideep",
            "hip",
            "ve",
            "fpga",
            "ort",
            "xla",
            "lazy",
            "vulkan",
            "mps",
            "meta",
            "hpu",
            "mtia",
        ],
    ),
    help="Device to run on. (Default is cuda)",
)
@click.option(
    "--show_sources",
    "-s",
    is_flag=True,
    help="Show sources along with answers (Default is False)",
)
@click.option(
    "--use_history",
    "-h",
    is_flag=True,
    help="Use history (Default is False)",
)
@click.option(
    "--model_type",
    default="llama",
    type=click.Choice(
        ["llama", "mistral", "non_llama", "llama-one-shot", "llama-few-shot", "mistral-few-shot","zephyr","zephyr-few-shot"],
    ),
    help="model type, llama, mistral or non_llama",
)
def main(device_type, show_sources, use_history, model_type):

    os.environ["OPENAI_API_KEY"] = "XXXXXXXXXXXXXXX" # https://platform.openai.com/account/api-keys
    assert os.environ.get("OPENAI_API_KEY") is not None, "Please set OPENAI_API_KEY environment variable"

    evaluation_llm = ChatOpenAI(model="gpt-4", request_timeout=120)

    logging.info(f"Running on: {device_type}")
    logging.info(f"Display Source Documents set to: {show_sources}")
    logging.info(f"Use history set to: {use_history}")

    # check if models directory do not exist, create a new one and store models here.
    if not os.path.exists(MODELS_PATH):
        os.mkdir(MODELS_PATH)

    for example in eval_examples:
        example["query"] = ' '.join(example["query"].split())
        example["answer"] = ' '.join(example["answer"].split()).strip()

    qa, prompt, llm = retrieval_qa_pipline(device_type, use_history, promptTemplate_type=model_type)
    print(prompt)

    predictions = qa.apply(eval_examples)

    # assisted evaluation
    eval_chain = QAEvalChain.from_llm(evaluation_llm)
    graded_outputs = eval_chain.evaluate(eval_examples, predictions)

    evaluator = load_evaluator("labeled_score_string", criteria=criteria, llm=evaluation_llm, normalize_by=10)

    for i, value in enumerate(graded_outputs):
        print(f"Element {i}: {value}")

    for i, eg in enumerate(eval_examples):
        print(f"Example {i}:")
        print("Question: " + predictions[i]['query'])
        print("Real Answer: " + predictions[i]['answer'])
        print("Predicted Answer: " + predictions[i]['result'])
        print("Predicted Grade: " + graded_outputs[i]['results'])
        print()
        print("----------------------------------------")
        eval_result = evaluator.evaluate_strings(prediction=predictions[i]['result'], reference=predictions[i]['answer'], input=predictions[i]['query'])
        print(eval_result)
        print("----------------------------------------")
        #print(eval_result['reasoning'].split('.'))



if __name__ == "__main__":
    logging.basicConfig(
        format="%(asctime)s - %(levelname)s - %(filename)s:%(lineno)s - %(message)s", level=logging.INFO
    )
    main()
