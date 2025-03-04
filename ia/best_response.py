from wikihow_research import *
from wikipedia_reseach import *
from solve_mathematics import *

def wiki_response(question):
    if is_math_expression(question):
        return {"text": solve_math_expression(question), "source": "Math Solver", "score": 0}
    
    response_wikipedia = rechercher_wikipedia(question)
    response_wikihow = rechercher_wikihow(question)

    if response_wikipedia["score"] > response_wikihow["score"]:
        return response_wikipedia
    else:
        return response_wikihow
    
